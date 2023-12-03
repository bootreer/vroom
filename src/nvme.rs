use crate::{
    cmd::NvmeCommand,
    pci::{pci_map_bar, pci_map_resource, PciResource},
    queues::*,
};
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
    // bar: PciResource,
    // Doorbell stride
    dstrd: u16,
    admin_sq: NvmeSubQueue,
    admin_cq: NvmeCompQueue,
    sub_queues: Vec<NvmeSubQueue>,
    comp_queues: Vec<NvmeCompQueue>,
    // namespaces: Vec<NvmeNamespace>
    // buffer: Dma<[u8; 2048 * 1024]>, // 2MiB of buffer
}

impl NvmeDevice {
    pub fn init(pci_addr: &str) -> Result<Self, Box<dyn Error>> {
        let (addr, len) = pci_map_resource(pci_addr)?;
        println!("length of mapped resource: {len}");

        // TODO: read bar data
        // TODO: init admin queues
        let admin_sq = NvmeSubQueue::new()?;
        let admin_cq = NvmeCompQueue::new()?;

        println!("test");
        let mut dev = Self {
            pci_addr: pci_addr.to_string(),
            // fix
            dstrd: 0, /*  {
                          unsafe {
                              ((std::ptr::read_volatile((bar.addr as usize + NvmeRegs64::CAP as usize) as *const u64) >> 32) & 0b1111) as u16
                          }
                      }, */
            addr,
            len,
            admin_sq,
            admin_cq,
            sub_queues: vec![],
            comp_queues: vec![],
        };

        {
            let cap = dev.get_reg64(NvmeRegs64::CAP as u64);
            println!("capabilities: 0x{:x}", cap);

            dev.dstrd = ((dev.get_reg64(NvmeRegs64::CAP as u64) << 32) & 0b1111) as u16;
            println!("dev.dstrd: {:x}", dev.dstrd);

            println!("Setting admin queues");
            println!("asq get addr: {}", dev.admin_sq.get_addr());
            println!("asq get addr: {}", dev.admin_sq.commands.phys);
            dev.set_reg64(NvmeRegs64::ASQ as u32, dev.admin_sq.get_addr() as u64);
            let admin_q_s = dev.get_reg64(NvmeRegs64::ASQ as u64);
            dev.set_reg64(NvmeRegs64::ACQ as u32, dev.admin_cq.get_addr() as u64);
            let admin_q_c = dev.get_reg64(NvmeRegs64::ACQ as u64);
            println!("admin sq addr: {admin_q_s}, admin cq addr: {admin_q_c}");

            println!("Setting AQA");
            dev.set_reg32(
                NvmeRegs32::AQA as u32,
                (QUEUE_LENGTH as u32 - 1) << 16 | (QUEUE_LENGTH as u32 - 1),
            );
        }

        // TODO: init i/o queues

        Ok(dev)
    }

    pub fn identify_controller(&mut self) -> Result<(), Box<dyn Error>> {
        println!("trying to identify controller");
        let buffer: Dma<[u8; 2048]> = Dma::allocate(2048, false)?;
        {
            let entry = NvmeCommand::identify_controller(self.admin_sq.tail as u16, buffer.phys);
            let tail = self.admin_sq.submit(entry);
            self.write_reg_idx(NvmeArrayRegs::SQyTDBL, 0, tail as u32);
        }

        println!("trying to retrieve completion for command");
        {
            let (head, entry, _) = self.admin_cq.complete_spin();
            self.write_reg_idx(NvmeArrayRegs::CQyHDBL, 0, head as u32);
        }

        println!("dumping identify controller");

        let mut serial = String::new();
        let data = unsafe { *buffer.virt };

        for &b in &data[4..24] {
            if b == 0 {
                break;
            }
            serial.push(b as char);
        }

        let mut model = String::new();
        for &b in &data[24..64] {
            if b == 0 {
                break;
            }
            model.push(b as char);
        }

        let mut firmware = String::new();
        for &b in &data[64..72] {
            if b == 0 {
                break;
            }
            firmware.push(b as char);
        }

        println!(
            "  - Model: {} Serial: {} Firmware: {}",
            model.trim(),
            serial.trim(),
            firmware.trim()
        );

        Ok(())
    }

    fn write_reg_idx(&self, reg: NvmeArrayRegs, qid: u16, val: u32) {
        match reg {
            NvmeArrayRegs::SQyTDBL => unsafe {
                std::ptr::write_volatile(
                    (self.addr as usize + 0x1000 + ((4 << self.dstrd) * (2 * qid)) as usize)
                        as *mut u32,
                    val,
                );
            },
            NvmeArrayRegs::CQyHDBL => unsafe {
                std::ptr::write_volatile(
                    (self.addr as usize + 0x1000 + ((4 << self.dstrd) * (2 * qid + 1)) as usize)
                        as *mut u32,
                    val,
                );
            },
        }
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

    /// Returns the register at `self.addr` + `reg`.
    ///
    /// # Panics
    ///
    /// Panics if `self.addr` + `reg` does not belong to the mapped memory of the pci device.
    fn get_reg32(&self, reg: u32) -> u32 {
        assert!(reg as usize <= self.len - 4, "memory access out of bounds");

        unsafe { std::ptr::read_volatile((self.addr as usize + reg as usize) as *mut u32) }
    }

    /// Sets the register at `self.addr` + `reg` to `value`.
    ///
    /// # Panics
    ///
    /// Panics if `self.addr` + `reg` does not belong to the mapped memory of the pci device.
    fn set_reg64(&self, reg: u32, value: u64) {
        assert!(reg as usize <= self.len - 8, "memory access out of bounds");

        unsafe {
            std::ptr::write_volatile((self.addr as usize + reg as usize) as *mut u64, value);
        }
    }

    /// Returns the register at `self.addr` + `reg`.
    ///
    /// # Panics
    ///
    /// Panics if `self.addr` + `reg` does not belong to the mapped memory of the pci device.
    fn get_reg64(&self, reg: u64) -> u64 {
        assert!(reg as usize <= self.len - 8, "memory access out of bounds");

        unsafe { std::ptr::read_volatile((self.addr as usize + reg as usize) as *mut u64) }
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
