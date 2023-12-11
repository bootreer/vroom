use crate::cmd::NvmeCommand;
use crate::memory::Dma;
use crate::pci::pci_map_resource;
use crate::queues::*;
use crate::NvmeNamespace;
use libc::pause;
use std::error::Error;

// clippy doesnt like this
#[allow(non_camel_case_types, unused)]
#[derive(Copy, Clone, Debug)]
pub enum NvmeRegs32 {
    VS = 0x8,        // Version
    INTMS = 0xC,     // Interrupt Mask Set
    INTMC = 0x10,    // Interrupt Mask Clear
    CC = 0x14,       // Controller Configuration
    CSTS = 0x1C,     // Controller Status
    NSSR = 0x20,     // NVM Subsystem Reset
    AQA = 0x24,      // Admin Queue Attributes
    CMBLOC = 0x38,   // Contoller Memory Buffer Location
    CMBSZ = 0x3C,    // Controller Memory Buffer Size
    BPINFO = 0x40,   // Boot Partition Info
    BPRSEL = 0x44,   // Boot Partition Read Select
    BPMBL = 0x48,    // Bood Partition Memory Location
    CMBSTS = 0x58,   // Controller Memory Buffer Status
    PMRCAP = 0xE00,  // PMem Capabilities
    PMRCTL = 0xE04,  // PMem Region Control
    PMRSTS = 0xE08,  // PMem Region Status
    PMREBS = 0xE0C,  // PMem Elasticity Buffer Size
    PMRSWTP = 0xE10, // PMem Sustained Write Throughput
}

#[allow(non_camel_case_types, unused)]
#[derive(Copy, Clone, Debug)]
pub enum NvmeRegs64 {
    CAP = 0x0,      // Controller Capabilities
    ASQ = 0x28,     // Admin Submission Queue Base Address
    ACQ = 0x30,     // Admin Completion Queue Base Address
    CMBMSC = 0x50,  // Controller Memory Buffer Space Control
    PMRMSC = 0xE14, // Persistent Memory Buffer Space Control
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
pub(crate) enum NvmeArrayRegs {
    SQyTDBL,
    CQyHDBL,
}

// who tf is abbreviating this stuff
#[repr(packed)]
#[derive(Debug, Clone, Copy)]
#[allow(unused)]
struct IdentifyNamespaceData {
    pub nsze: u64,
    pub ncap: u64,
    nuse: u64,
    nsfeat: u8,
    pub nlbaf: u8,
    pub flbas: u8,
    mc: u8,
    dpc: u8,
    dps: u8,
    nmic: u8,
    rescap: u8,
    fpi: u8,
    dlfeat: u8,
    nawun: u16,
    nawupf: u16,
    nacwu: u16,
    nabsn: u16,
    nabo: u16,
    nabspf: u16,
    noiob: u16,
    nvmcap: u128,
    npwg: u16,
    npwa: u16,
    npdg: u16,
    npda: u16,
    nows: u16,
    _rsvd1: [u8; 18],
    anagrpid: u32,
    _rsvd2: [u8; 3],
    nsattr: u8,
    nvmsetid: u16,
    endgid: u16,
    nguid: [u8; 16],
    eui64: u64,
    pub lba_format_support: [u32; 16],
    _rsvd3: [u8; 192],
    vendor_specific: [u8; 3712],
}

pub struct NvmeDevice {
    pci_addr: String,
    addr: *mut u8,
    len: usize,
    // Doorbell stride
    dstrd: u16,
    admin_sq: NvmeSubQueue,
    admin_cq: NvmeCompQueue,
    sub_queues: Vec<NvmeSubQueue>,
    comp_queues: Vec<NvmeCompQueue>,
    buffer: Dma<[u8; 2048 * 1024]>, // 2MiB of buffer
    namespaces: Vec<NvmeNamespace>,
}

impl NvmeDevice {
    pub fn init(pci_addr: &str) -> Result<Self, Box<dyn Error>> {
        let (addr, len) = pci_map_resource(pci_addr)?;
        let dev = Self {
            pci_addr: pci_addr.to_string(),
            addr,
            dstrd: {
                unsafe {
                    ((std::ptr::read_volatile(
                        (addr as usize + NvmeRegs64::CAP as usize) as *const u64,
                    ) >> 32)
                        & 0b1111) as u16
                }
            },
            len,
            admin_sq: NvmeSubQueue::new()?,
            admin_cq: NvmeCompQueue::new()?,
            sub_queues: vec![],
            comp_queues: vec![],
            buffer: Dma::allocate(crate::memory::HUGE_PAGE_SIZE, true)?,
            namespaces: vec![],
        };

        println!("CAP: 0x{:x}", dev.get_reg64(NvmeRegs64::CAP as u64));
        println!("VS: 0x{:x}", dev.get_reg32(NvmeRegs32::VS as u32));
        println!("CC: 0x{:x}", dev.get_reg32(NvmeRegs32::CC as u32));

        println!("Disabling controller");
        // Set Enable bit to 0
        let ctrl_config = dev.get_reg32(NvmeRegs32::CC as u32) & 0xFFFF_FFFE;
        dev.set_reg32(NvmeRegs32::CC as u32, ctrl_config);

        // Wait for not ready
        loop {
            let csts = dev.get_reg32(NvmeRegs32::CSTS as u32);
            if csts & 1 == 1 {
                unsafe {
                    pause();
                }
            } else {
                break;
            }
        }

        // Configure Admin Queues
        dev.set_reg64(NvmeRegs64::ASQ as u32, dev.admin_sq.get_addr() as u64);
        dev.set_reg64(NvmeRegs64::ACQ as u32, dev.admin_cq.get_addr() as u64);
        dev.set_reg32(
            NvmeRegs32::AQA as u32,
            (QUEUE_LENGTH as u32 - 1) << 16 | (QUEUE_LENGTH as u32 - 1),
        );

        // Configure other stuff
        // TODO: check css values
        let mut cc = dev.get_reg32(NvmeRegs32::CC as u32);
        // mask out reserved stuff
        cc &= 0xFF00_000F;
        // Set Completion (2^4 = 16 Bytes) and Submission Entry (2^6 = 64 Bytes) sizes
        cc |= (4 << 20) | (6 << 16);
        dev.set_reg32(NvmeRegs32::CC as u32, cc);

        // Enable the controller
        println!("Enabling controller");
        let ctrl_config = dev.get_reg32(NvmeRegs32::CC as u32) | 1;
        dev.set_reg32(NvmeRegs32::CC as u32, ctrl_config);

        // wait for ready
        loop {
            let csts = dev.get_reg32(NvmeRegs32::CSTS as u32);
            if csts & 1 == 0 {
                unsafe {
                    pause();
                }
            } else {
                break;
            }
        }
        Ok(dev)
    }

    pub fn identify_controller(&mut self) -> Result<(), Box<dyn Error>> {
        println!("Trying to identify controller");
        let _entry =
            self.submit_and_complete_admin(|c_id, ptr| NvmeCommand::identify_controller(c_id, ptr));

        println!("Dumping identify controller");
        let mut serial = String::new();
        let data = unsafe { *self.buffer.virt };

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

    pub fn create_io_queue_pair(&mut self) -> Result<(), Box<dyn Error>> {
        println!("Requesting i/o completion queue");
        let cq_id = self.comp_queues.len();
        let queue = NvmeCompQueue::new()?;
        self.submit_and_complete_admin(|c_id, _| {
            NvmeCommand::create_io_completion_queue(
                c_id,
                cq_id as u16,
                queue.get_addr(),
                (QUEUE_LENGTH - 1) as u16,
            )
        });
        self.comp_queues.push(queue);

        println!("Requesting i/o submission queue");
        let queue = NvmeSubQueue::new()?;
        let q_id = self.sub_queues.len();
        self.submit_and_complete_admin(|c_id, _| {
            NvmeCommand::create_io_submission_queue(
                c_id,
                q_id as u16,
                queue.get_addr(),
                (QUEUE_LENGTH - 1) as u16,
                cq_id as u16,
            )
        });
        self.sub_queues.push(queue);

        Ok(())
    }

    pub fn identify_namespace_list(&mut self, base: u32) -> Vec<u32> {
        self.submit_and_complete_admin(|c_id, addr| {
            NvmeCommand::identify_namespace_list(c_id, addr, base)
        });

        // TODO: idk bout this/don't hardcode len
        let data: &[u32] =
            unsafe { std::slice::from_raw_parts(self.buffer.virt as *const u32, 1024) };

        data.iter()
            .copied()
            .take_while(|&id| id != 0)
            .collect::<Vec<u32>>()
    }

    pub fn identify_namespace(&mut self, id: u32) -> NvmeNamespace {
        // TODO: use self.buffer
        let tmp_buff: Dma<IdentifyNamespaceData> =
            Dma::allocate(std::mem::size_of::<IdentifyNamespaceData>(), true).unwrap();

        self.submit_and_complete_admin(|c_id, _| {
            NvmeCommand::identify_namespace(c_id, tmp_buff.phys, id)
        });

        let namespace_data = unsafe { *tmp_buff.virt };
        let size = namespace_data.nsze;
        let blocks = namespace_data.ncap;

        // figure out block size
        let flba_idx = (namespace_data.flbas & 0xF) as usize;
        let flba_data = (namespace_data.lba_format_support[flba_idx] >> 16) & 0xFF;
        let block_size = if flba_data < 9 || flba_data >= 32 {
            0
        } else {
            1 << flba_data
        };

        // TODO: check metadata?

        println!("Namespace {id}, Size: {size}, Blocks: {blocks}, Block size: {block_size}");

        NvmeNamespace {
            id,
            blocks,
            block_size,
        }
    }

    pub fn submit_and_complete_admin<F: FnOnce(u16, usize) -> NvmeCommand>(
        &mut self,
        cmd_init: F,
    ) -> NvmeCompletion {
        let cid = self.admin_sq.tail;
        let tail = self.admin_sq.submit(cmd_init(cid as u16, self.buffer.phys));
        self.write_reg_idx(NvmeArrayRegs::SQyTDBL, 0, tail as u32);

        let (head, entry, _) = self.admin_cq.complete_spin();
        self.write_reg_idx(NvmeArrayRegs::CQyHDBL, 0, head as u32);
        entry
    }

    /// Sets Queue `qid` Tail Doorbell to `val`
    fn write_reg_idx(&self, reg: NvmeArrayRegs, qid: u16, val: u32) {
        // TODO: catch invalid qid i guess
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
