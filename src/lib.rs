#![no_std]
#![warn(missing_docs)]

//! An embedded async driver for the MAX14578E/MAX14578AE USB battery charger detectors.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use modular_bitfield::{bitfield, specifiers::B1, BitfieldSpecifier};

const ADDR: u8 = 0x2c;

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
struct Reg(pub u8);

#[allow(dead_code)]
impl Reg {
    pub const DEVICE_ID: Reg = Reg(0x00);
    pub const CONTROL_1: Reg = Reg(0x01);
    pub const INTERRUPT: Reg = Reg(0x02);
    pub const CONTROL_2: Reg = Reg(0x03);
    pub const CONTROL_3: Reg = Reg(0x04);

    pub const fn new(val: u8) -> Self {
        Reg(val)
    }

    pub const fn to_u8(self) -> u8 {
        self.0
    }
}

/// A MAX14578E/MAX14578AE USB charger detector
pub struct ChargerDetector<D> {
    i2c_dev: D,
}

impl<D: I2c> ChargerDetector<D> {
    /// Create a new `ChargerDetector`
    pub fn new(i2c_dev: D) -> Self {
        ChargerDetector { i2c_dev }
    }

    /// Get the device ID. Should return `0b0010_0001`.
    pub async fn device_id(&mut self) -> Result<u8, D::Error> {
        self.read_reg(Reg::DEVICE_ID).await
    }

    /// Enable the /INT pin of the charger detector
    pub async fn enable_irq(&mut self, enable: bool) -> Result<(), D::Error> {
        const INTEN: u8 = 0x40;
        self.modify_reg(Reg::CONTROL_1, |x| {
            x & !INTEN | if enable { INTEN } else { 0 }
        })
        .await
    }

    /// Read and clear the current interrupt status flags
    pub async fn irq_status(&mut self) -> Result<InterruptStatus, D::Error> {
        self.read_reg(Reg::INTERRUPT)
            .await
            .map(|x| InterruptStatus::from_bytes([x]))
    }

    /// Close or open the USB switch.
    ///
    /// The switch should be closed after a USB data port is detected and re-opened when the data port is detached.
    pub async fn set_usb_switch<T: DelayNs>(
        &mut self,
        closed: bool,
        mut delay: T,
    ) -> Result<(), D::Error> {
        const CP_ENA: u8 = 0x10;
        const USBSWC: u8 = 0x20;

        if closed {
            self.modify_reg(Reg::CONTROL_1, |x| x | CP_ENA).await?;
            delay.delay_ms(1).await;
            self.modify_reg(Reg::CONTROL_1, |x| x | USBSWC).await
        } else {
            self.modify_reg(Reg::CONTROL_1, |x| x & !USBSWC).await?;
            delay.delay_ms(1).await;
            self.modify_reg(Reg::CONTROL_1, |x| x & !CP_ENA).await
        }
    }

    async fn read_reg(&mut self, reg: Reg) -> Result<u8, D::Error> {
        let mut val = 0u8;
        self.i2c_dev
            .write_read(
                ADDR,
                core::slice::from_ref(&reg.to_u8()),
                core::slice::from_mut(&mut val),
            )
            .await?;
        Ok(val)
    }

    async fn write_reg(&mut self, reg: Reg, val: u8) -> Result<(), D::Error> {
        let buf = [reg.to_u8(), val];
        self.i2c_dev.write(ADDR, &buf).await
    }

    async fn modify_reg<F: FnOnce(u8) -> u8>(&mut self, reg: Reg, func: F) -> Result<(), D::Error> {
        let val = self.read_reg(reg).await?;
        let val = func(val);
        self.write_reg(reg, val).await
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[bits = 3]
/// The type of charger that has been detected
pub enum ChargerType {
    /// Nothing attached
    #[default]
    None,
    /// USB cable attached
    Usb,
    /// USB BC1.1 compatible charging downstream port
    ChargingDownstreamPort,
    /// USB BC1.1 dedicated charging port
    DedicatedChargingPort,
    /// Special charger: 500mA max
    Special500mA,
    /// Special charger: 1A max
    Special1000mA,
}

impl ChargerType {
    /// Is a USB data port attached?
    pub fn is_usb(self) -> bool {
        matches!(self, ChargerType::Usb | ChargerType::ChargingDownstreamPort)
    }

    /// Can the attached port provide charge?
    pub fn can_charge(self) -> bool {
        !matches!(self, ChargerType::None)
    }

    /// Indicates a (conservative) current limit for the attached port
    ///
    /// `Usb` ports can provide up to 500mA after negotiation. `ChargingDownstreamPort` and
    /// `DedicatedChargingPort` may provide up to 5A, but are not required to and may
    /// reduce voltage or shutdown if their current limit is exceeded. A `DedicatedChargingPort`
    /// is allowed to have a current limit as low as 500mA.
    pub fn current_limit(self) -> u16 {
        match self {
            ChargerType::None => 0,
            ChargerType::Usb => 100,                     // 100mA to 500mA
            ChargerType::ChargingDownstreamPort => 1500, // 1.5A to 5.0A
            ChargerType::DedicatedChargingPort => 1500,  // 0.5A to 5.0A
            ChargerType::Special500mA => 500,
            ChargerType::Special1000mA => 1000,
        }
    }
}

#[bitfield(bits = 8)]
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, BitfieldSpecifier)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
/// Charger detector status flags
pub struct InterruptStatus {
    #[skip]
    __: B1,
    pub is_charger_detection_running: bool,
    pub is_data_contact_detection_running: bool,
    #[skip]
    __: B1,
    pub is_vbus_present: bool,
    #[bits = 3]
    pub charger_type: ChargerType,
}
