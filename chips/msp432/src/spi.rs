use crate::dma;
use crate::usci::{self, UCSPIxCTLW0, UCSPIxIE, UCSPIxIFG, UCSPIxSTATW};
use core::cell::Cell;
use kernel::common::{
    cells::{OptionalCell, TakeCell},
    registers::{ReadOnly, ReadWrite},
    StaticRef,
};
use kernel::hil::gpio::{Output, Pin};
use kernel::hil::spi;
use kernel::ReturnCode;

/// The SPI related registers are identical between the USCI_A and the USCI_B module, but the usage
/// of the certain bits is different to UART and I2C mode. Thus we simply cast the concerned
/// registers to a UCSPIx-registers instead of UCAx or UCBx registers. With this trick we can pass
/// references of USCI_A and USCI_B modules to the Spi-constructor.
pub trait UsciSpiRef {
    fn ctlw0(&self) -> &ReadWrite<u16, UCSPIxCTLW0::Register>;
    fn brw(&self) -> &ReadWrite<u16>;
    fn statw(&self) -> &ReadWrite<u16, UCSPIxSTATW::Register>;
    fn rxbuf(&self) -> &ReadOnly<u16>;
    fn txbuf(&self) -> &ReadWrite<u16>;
    fn ie(&self) -> &ReadWrite<u16, UCSPIxIE::Register>;
    fn ifg(&self) -> &ReadWrite<u16, UCSPIxIFG::Register>;
}

impl UsciSpiRef for StaticRef<usci::UsciBRegisters> {
    fn ctlw0(&self) -> &ReadWrite<u16, usci::UCSPIxCTLW0::Register> {
        unsafe {
            &*(&self.ctlw0 as *const ReadWrite<u16, usci::UCBxCTLW0::Register>
                as *const ReadWrite<u16, usci::UCSPIxCTLW0::Register>)
        }
    }

    fn brw(&self) -> &ReadWrite<u16> {
        &self.brw
    }

    fn statw(&self) -> &ReadWrite<u16, usci::UCSPIxSTATW::Register> {
        unsafe {
            &*(&self.statw as *const ReadWrite<u16, usci::UCBxSTATW::Register>
                as *const ReadWrite<u16, usci::UCSPIxSTATW::Register>)
        }
    }

    fn rxbuf(&self) -> &ReadOnly<u16> {
        &self.rxbuf
    }

    fn txbuf(&self) -> &ReadWrite<u16> {
        &self.txbuf
    }

    fn ie(&self) -> &ReadWrite<u16, usci::UCSPIxIE::Register> {
        unsafe {
            &*(&self.ie as *const ReadWrite<u16, usci::UCBxIE::Register>
                as *const ReadWrite<u16, usci::UCSPIxIE::Register>)
        }
    }

    fn ifg(&self) -> &ReadWrite<u16, usci::UCSPIxIFG::Register> {
        unsafe {
            &*(&self.ifg as *const ReadWrite<u16, usci::UCBxIFG::Register>
                as *const ReadWrite<u16, usci::UCSPIxIFG::Register>)
        }
    }
}

impl UsciSpiRef for StaticRef<usci::UsciARegisters> {
    fn ctlw0(&self) -> &ReadWrite<u16, usci::UCSPIxCTLW0::Register> {
        unsafe {
            &*(&self.ctlw0 as *const ReadWrite<u16, usci::UCAxCTLW0::Register>
                as *const ReadWrite<u16, usci::UCSPIxCTLW0::Register>)
        }
    }

    fn brw(&self) -> &ReadWrite<u16> {
        &self.brw
    }

    fn statw(&self) -> &ReadWrite<u16, usci::UCSPIxSTATW::Register> {
        unsafe {
            &*(&self.statw as *const ReadWrite<u16, usci::UCAxSTATW::Register>
                as *const ReadWrite<u16, usci::UCSPIxSTATW::Register>)
        }
    }

    fn rxbuf(&self) -> &ReadOnly<u16> {
        &self.rxbuf
    }

    fn txbuf(&self) -> &ReadWrite<u16> {
        &self.txbuf
    }

    fn ie(&self) -> &ReadWrite<u16, usci::UCSPIxIE::Register> {
        unsafe {
            &*(&self.ie as *const ReadWrite<u16, usci::UCAxIE::Register>
                as *const ReadWrite<u16, usci::UCSPIxIE::Register>)
        }
    }

    fn ifg(&self) -> &ReadWrite<u16, usci::UCSPIxIFG::Register> {
        unsafe {
            &*(&self.ifg as *const ReadWrite<u16, usci::UCAxIFG::Register>
                as *const ReadWrite<u16, usci::UCSPIxIFG::Register>)
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum TransmissionType {
    Idle,
    Write,
    WriteRead,
}

pub struct Spi<'a> {
    registers: &'static dyn UsciSpiRef,
    cs: OptionalCell<&'a dyn Pin>,
    hold_cs: Cell<bool>,
    transmission_type: Cell<TransmissionType>,
    tx_buf: TakeCell<'static, [u8]>,
    master_client: OptionalCell<&'a dyn spi::SpiMasterClient>,

    tx_dma: OptionalCell<&'a dma::DmaChannel<'a>>,
    pub(crate) tx_dma_chan: usize,
    tx_dma_src: u8,

    rx_dma: OptionalCell<&'a dma::DmaChannel<'a>>,
    pub(crate) rx_dma_chan: usize,
    rx_dma_src: u8,
}

impl<'a> Spi<'a> {
    pub fn new(
        registers: &'static dyn UsciSpiRef,
        tx_dma_chan: usize,
        rx_dma_chan: usize,
        tx_dma_src: u8,
        rx_dma_src: u8,
    ) -> Self {
        Self {
            registers: registers,
            cs: OptionalCell::empty(),
            hold_cs: Cell::new(false),
            transmission_type: Cell::new(TransmissionType::Write),
            tx_buf: TakeCell::empty(),
            master_client: OptionalCell::empty(),

            tx_dma: OptionalCell::empty(),
            tx_dma_chan: tx_dma_chan,
            tx_dma_src: tx_dma_src,

            rx_dma: OptionalCell::empty(),
            rx_dma_chan: rx_dma_chan,
            rx_dma_src: rx_dma_src,
        }
    }

    pub fn set_dma(&self, tx_dma: &'a dma::DmaChannel<'a>, rx_dma: &'a dma::DmaChannel<'a>) {
        self.tx_dma.replace(tx_dma);
        self.rx_dma.replace(rx_dma);
    }

    fn set_module_to_reset(&self) {
        // Set module to reset in order to enable the configuration
        self.registers.ctlw0().modify(UCSPIxCTLW0::UCSWRST::Enabled);
    }

    fn clear_module_reset(&self) {
        self.registers
            .ctlw0()
            .modify(UCSPIxCTLW0::UCSWRST::Disabled);
    }

    fn finish_transfer(&self) {
        self.transmission_type.set(TransmissionType::Idle);

        if !self.hold_cs.get() {
            self.cs.map(|pin| pin.set());
        }
    }
}

impl<'a> dma::DmaClient for Spi<'a> {
    fn transfer_done(
        &self,
        tx_buf: Option<&'static mut [u8]>,
        rx_buf: Option<&'static mut [u8]>,
        transmitted_bytes: usize,
    ) {
        if let Some(buf) = tx_buf {
            // Transmitting finished
            if self.transmission_type.get() == TransmissionType::Write {
                // Only a write operation was done -> invoke callback

                self.finish_transfer();
                self.master_client
                    .map(move |cl| cl.read_write_done(buf, None, transmitted_bytes));
            } else {
                // Also a read operation was done -> wait for RX callback
                self.tx_buf.replace(buf);
            }
        }

        if let Some(buf) = rx_buf {
            // Receiving finished

            self.finish_transfer();
            self.master_client.map(move |cl| {
                cl.read_write_done(
                    self.tx_buf
                        .take()
                        .unwrap_or_else(|| panic!("SPI: no TX buffer was returned from DMA")),
                    Some(buf),
                    transmitted_bytes,
                )
            });
        }
    }
}

impl<'a> spi::SpiMaster for Spi<'a> {
    type ChipSelect = &'a dyn Pin;

    fn set_client(&self, client: &'static dyn spi::SpiMasterClient) {
        self.master_client.set(client);
    }

    fn init(&self) {
        self.set_module_to_reset();

        self.registers.ctlw0().modify(
            // Transmit LSB first
            UCSPIxCTLW0::UCMSB::LSBFirst
            // Enable 8bit modus
            + UCSPIxCTLW0::UC7BIT::_8Bit
            // Configure to Master mode
            + UCSPIxCTLW0::UCMST::Master
            // Use 3-pin SPI mode since CS is controlled by hand
            + UCSPIxCTLW0::UCMODE::_3PinSPI
            // Enable synchronous mode
            + UCSPIxCTLW0::UCSYNC::SynchronousMode
            // Use SMCLK as clock
            + UCSPIxCTLW0::UCSSEL::SMCLK,
        );

        // Configure the DMA
        let tx_conf = dma::DmaConfig {
            src_chan: self.tx_dma_src,
            mode: dma::DmaMode::Basic,
            width: dma::DmaDataWidth::Width8Bit,
            src_incr: dma::DmaPtrIncrement::Incr8Bit,
            dst_incr: dma::DmaPtrIncrement::NoIncr,
        };

        let rx_conf = dma::DmaConfig {
            src_chan: self.rx_dma_src,
            mode: dma::DmaMode::Basic,
            width: dma::DmaDataWidth::Width8Bit,
            src_incr: dma::DmaPtrIncrement::NoIncr,
            dst_incr: dma::DmaPtrIncrement::Incr8Bit,
        };

        self.tx_dma.map(|dma| dma.initialize(&tx_conf));
        self.rx_dma.map(|dma| dma.initialize(&rx_conf));

        self.clear_module_reset();
    }

    fn is_busy(&self) -> bool {
        self.transmission_type.get() != TransmissionType::Idle
    }

    fn read_write_bytes(
        &self,
        write_buffer: &'static mut [u8],
        read_buffer: Option<&'static mut [u8]>,
        len: usize,
    ) -> ReturnCode {
        if self.is_busy() {
            return ReturnCode::EBUSY;
        }

        let mut cnt = len;

        // Set chip select
        self.cs.map(|pin| pin.clear());

        // If a read-buffer was supplied too, we also start a read transaction
        if let Some(read_buf) = read_buffer {
            self.transmission_type.set(TransmissionType::WriteRead);
            cnt = core::cmp::min(read_buf.len(), write_buffer.len());

            let rx_reg = self.registers.rxbuf() as *const ReadOnly<u16> as *const ();
            self.rx_dma
                .map(move |dma| dma.transfer_periph_to_mem(rx_reg, read_buf, cnt));
        } else {
            self.transmission_type.set(TransmissionType::Write);
        }

        // Start a write transaction
        let tx_reg = self.registers.txbuf() as *const ReadWrite<u16> as *const ();
        self.tx_dma
            .map(move |dma| dma.transfer_mem_to_periph(tx_reg, write_buffer, cnt));

        ReturnCode::SUCCESS
    }

    fn write_byte(&self, val: u8) {
        todo!()
    }

    fn read_byte(&self) -> u8 {
        todo!()
    }

    fn read_write_byte(&self, val: u8) -> u8 {
        todo!()
    }

    fn specify_chip_select(&self, cs: Self::ChipSelect) {
        cs.make_output();
        cs.set();
        self.cs.set(cs);
    }

    fn set_rate(&self, rate: u32) -> u32 {
        todo!()
    }

    fn get_rate(&self) -> u32 {
        todo!()
    }

    fn set_clock(&self, polarity: spi::ClockPolarity) {
        self.set_module_to_reset();

        match polarity {
            spi::ClockPolarity::IdleLow => self
                .registers
                .ctlw0()
                .modify(UCSPIxCTLW0::UCCKPL::InactiveLow),
            spi::ClockPolarity::IdleHigh => self
                .registers
                .ctlw0()
                .modify(UCSPIxCTLW0::UCCKPL::InactiveHigh),
        }

        self.clear_module_reset();
    }

    fn get_clock(&self) -> spi::ClockPolarity {
        match self.registers.ctlw0().is_set(UCSPIxCTLW0::UCCKPL) {
            false => spi::ClockPolarity::IdleLow,
            true => spi::ClockPolarity::IdleHigh,
        }
    }

    fn set_phase(&self, phase: spi::ClockPhase) {
        self.set_module_to_reset();

        match phase {
            spi::ClockPhase::SampleLeading => self
                .registers
                .ctlw0()
                .modify(UCSPIxCTLW0::UCCKPH::CaptureFirstChangeFollowing),
            spi::ClockPhase::SampleTrailing => self
                .registers
                .ctlw0()
                .modify(UCSPIxCTLW0::UCCKPH::ChangeFirstCaptureFollowing),
        }

        self.clear_module_reset();
    }

    fn get_phase(&self) -> spi::ClockPhase {
        match self.registers.ctlw0().is_set(UCSPIxCTLW0::UCCKPH) {
            false => spi::ClockPhase::SampleTrailing,
            true => spi::ClockPhase::SampleLeading,
        }
    }

    fn hold_low(&self) {
        self.hold_cs.set(true);
    }

    fn release_low(&self) {
        self.hold_cs.set(false);
    }
}
