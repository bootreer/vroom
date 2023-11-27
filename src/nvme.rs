use crate::{pci::pci_map_resource, queues::*};
use std::error::Error;

#[allow(unused)]
use crate::memory::Dma;

// clippy doesnt like this
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub enum NvmeRegs32 {
    VS = 0x8,
    INTMS = 0xC,
    INTMC = 0x10,
    CC = 0x14,
    CSTS = 0x1C,
    NSSR = 0x20,
    AQA = 0x24,
    CMBLOC = 0x38,
    CMBSZ = 0x3C,
    BPINFO = 0x40,
    BPRSEL = 0x44,
    BPMBL = 0x48,
    CMBSTS = 0x58,
    PMRCAP = 0xE00,
    PMRCTL = 0xE04,
    PMRSTS = 0xE08,
    PMREBS = 0xE0C,
    PMRSWTP = 0xE10,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub enum NvmeRegs64 {
    CAP = 0x0,
    ASQ = 0x28,
    ACQ = 0x30,
    CMBMSC = 0x50,
    PMRMSC = 0xE14,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub(crate) enum NvmeArrayRegs {
    SQyTDBL,
    CQyHDBL,
}

pub struct NvmeDevice {
    pci_addr: String,
    addr: *mut u8,
    len: usize,
    sub_queues: Vec<NvmeSubQueue>,
    comp_queues: Vec<NvmeCompQueue>,
    // namespaces: Vec<NvmeNamespace>
    // interrupt_method: InterruptMethod
    // buffer: Dma<[u8; 2048 * 1024]>, // 2MiB of buffer
}

impl NvmeDevice {
    #[allow(unused)]
    pub fn init(pci_addr: &str) -> Result<Self, Box<dyn Error>> {
        let (addr, len) = pci_map_resource(pci_addr)?;

        // TODO: read bar data

        let mut dev = Self {
            pci_addr: pci_addr.to_string(),
            addr,
            len,
            sub_queues: vec![],
            comp_queues: vec![],
            // TODO: tmp
            // interrupt_method: InterruptMethod::MsiX(MsixCfg {}),
        };

        // TODO: init admin queues
        let mut admin_sq = NvmeSubQueue::new()?;
        let mut admin_cq = NvmeCompQueue::new()?;

        // test
        dev.set_reg64(NvmeRegs64::ASQ as u32, admin_sq.get_addr() as u64);
        dev.set_reg64(NvmeRegs64::ACQ as u32, admin_cq.get_addr() as u64);

        dev.sub_queues.push(admin_sq);
        dev.comp_queues.push(admin_cq);

        // TODO: init i/o queues

        Ok(dev)
    }

    /// Sets the register at `self.addr` + `reg` to `value`.
    ///
    /// # Panics
    ///
    /// Panics if `self.addr` + `reg` does not belong to the mapped memory of the pci device.
    fn set_reg32(&self, reg: u32, value: u32) {
        assert!(reg as usize <= self.len - 4, "memory access out of bounds");

        unsafe {
            std::ptr::write_volatile((self.addr as usize + reg as usize) as *mut u32, value);
        }
    }

    /// Sets the register at `self.addr` + `reg` to `value`.
    ///
    /// # Panics
    ///
    /// Panics if `self.addr` + `reg` does not belong to the mapped memory of the pci device.
    fn set_reg64(&self, reg: u32, value: u64) {
        assert!(reg as usize <= self.len - 4, "memory access out of bounds");

        unsafe {
            std::ptr::write_volatile((self.addr as usize + reg as usize) as *mut u64, value);
        }
    }
}

// I guess ignore interrupts?
/// The way interrupts are sent. Unlike other PCI-based interfaces, like XHCI, it doesn't seem like
/// NVME supports operating with interrupts completely disabled.
pub enum InterruptMethod {
    /// Traditional level-triggered, INTx# interrupt pins.
    Intx,
    /// Message signaled interrupts
    Msi(MsiCapability),
    /// Extended message signaled interrupts
    MsiX(MsixCfg),
}

// TODO: tmp
pub struct MsiCapability {}
pub struct MsixCfg {}
