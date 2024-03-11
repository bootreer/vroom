use crate::cmd::NvmeCommand;
use crate::memory::{Dma, DmaSlice};
use crate::{HUGE_PAGE_SIZE, NvmeNamespace, NvmeStats};
use crate::pci::pci_map_resource;
use crate::queues::*;
use std::collections::HashMap;
use std::error::Error;

// clippy doesnt like this
#[allow(unused, clippy::upper_case_acronyms)]
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

#[allow(unused, clippy::upper_case_acronyms)]
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

#[allow(unused)]
pub struct NvmeQueuePair {
    pub id: u16,
    pub sub_queue: NvmeSubQueue,
    comp_queue: NvmeCompQueue,
}

impl NvmeQueuePair {
    pub fn submit_io(&mut self, data: &impl DmaSlice, mut lba: u64, write: bool) { // -> (u32, usize) {

        for chunk in data.chunks(2 * 4096) {
            let blocks = (chunk.slice.len() as u64 + 512 - 1) / 512;

            let addr = chunk.phys_addr as u64;
            let bytes = blocks * 512;
            let ptr1 = if bytes <= 4096 {
                0
            } else {
                addr + 4096 // self.page_size
            };

            let entry = if write {
                NvmeCommand::io_write(self.id << 11 | self.sub_queue.tail as u16, 1, lba, blocks as u16 - 1, addr, ptr1)
            } else {
                NvmeCommand::io_read(self.id << 11 | self.sub_queue.tail as u16, 1, lba, blocks as u16 - 1, addr, ptr1)
            };

            let tail = self.sub_queue.submit(entry);
            unsafe {
                std::ptr::write_volatile(
                    self.sub_queue.doorbell as *mut u32,
                    tail as u32,
                );
            }

            lba += blocks;
        }
    }

    pub fn complete_io(&mut self, n: usize) -> Option<u16> {
        let (tail, c_entry, _) = self.comp_queue.complete_n(n);
        unsafe {
            std::ptr::write_volatile(
                self.comp_queue.doorbell as *mut u32,
                tail as u32,
            );
        }

        let status = c_entry.status >> 1;
        if status != 0 {
            eprintln!(
                "Status: 0x{:x}, Status Code 0x{:x}, Status Code Type: 0x{:x}",
                status,
                status & 0xFF,
                (status >> 8) & 0x7
            );
            eprintln!("{:?}", c_entry);
            return None;
        }
        Some(c_entry.sq_head)
    }

}


#[allow(unused)]
pub struct NvmeDevice {
    pci_addr: String,
    addr: *mut u8,
    len: usize,
    // Doorbell stride
    dstrd: u16,
    admin_sq: NvmeSubQueue,
    admin_cq: NvmeCompQueue,
    // maybe map?
    pub io_sq: NvmeSubQueue,
    pub io_cq: NvmeCompQueue,
    buffer: Dma<u8>, // 2MiB of buffer
    prp_list: Dma<[u64; 512]>, // Address of PRP's, devices doesn't necessarily support 2MiB page sizes; 8 Bytes * 512 = 4096
    pub namespaces: HashMap<u32, NvmeNamespace>,
    pub stats: NvmeStats,
    q_id: u16,
}

// TODO
unsafe impl Send for NvmeDevice {}
unsafe impl Sync for NvmeDevice {}

#[allow(unused)]
impl NvmeDevice {
    pub fn init(pci_addr: &str) -> Result<Self, Box<dyn Error>> {
        let (addr, len) = pci_map_resource(pci_addr)?;
        let mut dev = Self {
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
            admin_sq: NvmeSubQueue::new(QUEUE_LENGTH, 0)?,
            admin_cq: NvmeCompQueue::new(QUEUE_LENGTH, 0)?,
            io_sq: NvmeSubQueue::new(QUEUE_LENGTH, 0)?,
            io_cq: NvmeCompQueue::new(QUEUE_LENGTH, 0)?,
            buffer: Dma::allocate(crate::memory::HUGE_PAGE_SIZE, true)?,
            prp_list: Dma::allocate(8 * 512, true)?,
            namespaces: HashMap::new(),
            stats: NvmeStats::default(),
            q_id: 1
        };

        for i in 1..512 {
            dev.prp_list[i - 1] = (dev.buffer.phys + i * 4096) as u64;
        }

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
                    super::pause();
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

        // Set Memory Page Size
        // let mpsmax = ((dev.get_reg64(NvmeRegs64::CAP as u64) >> 52) & 0xF) as u32;
        // cc |= (mpsmax << 7);
        // println!("MPS {}", (cc >> 7) & 0xF);
        dev.set_reg32(NvmeRegs32::CC as u32, cc);

        // Enable the controller
        println!("Enabling controller");
        let ctrl_config = dev.get_reg32(NvmeRegs32::CC as u32) | 1;
        dev.set_reg32(NvmeRegs32::CC as u32, ctrl_config);

        // wait for ready
        loop {
            let csts = dev.get_reg32(NvmeRegs32::CSTS as u32);
            if csts & 1 == 0 {
                super::pause();
            } else {
                break;
            }
        }

        let q_id = dev.q_id;
        let addr = dev.io_cq.get_addr();
        println!("Requesting i/o completion queue");
        let comp = dev.submit_and_complete_admin(|c_id, _| {
            NvmeCommand::create_io_completion_queue(
                c_id,
                q_id,
                addr,
                (QUEUE_LENGTH - 1) as u16,
            )
        })?;
        let addr = dev.io_sq.get_addr();
        println!("Requesting i/o submission queue");
        let comp = dev.submit_and_complete_admin(|c_id, _| {
            NvmeCommand::create_io_submission_queue(
                c_id,
                q_id,
                addr,
                (QUEUE_LENGTH - 1) as u16,
                q_id,
            )
        })?;
        dev.q_id += 1;

        Ok(dev)
    }

    pub fn identify_controller(&mut self) -> Result<(), Box<dyn Error>> {
        println!("Trying to identify controller");
        let _entry = self.submit_and_complete_admin(NvmeCommand::identify_controller);

        println!("Dumping identify controller");
        let mut serial = String::new();
        let data = &self.buffer;

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

    // 1 to 1 Submission/Completion Queue Mapping
    // TODO: return qpair instead?
    pub fn create_io_queue_pair(&mut self, len: usize) -> Result<NvmeQueuePair, Box<dyn Error>> {
        let q_id = self.q_id;
        println!("Requesting i/o queue pair with id {q_id}");

        let dbl = self.addr as usize + 0x1000 + ((4 << self.dstrd) * (2 * q_id + 1) as usize);
        let comp_queue = NvmeCompQueue::new(len, dbl)?;
        let comp = self.submit_and_complete_admin(|c_id, _| {
            NvmeCommand::create_io_completion_queue(
                c_id,
                q_id,
                comp_queue.get_addr(),
                (len - 1) as u16,
            )
        })?;

        let dbl = self.addr as usize + 0x1000 + ((4 << self.dstrd) * (2 * q_id) as usize);
        let sub_queue = NvmeSubQueue::new(len, dbl)?;
        let comp = self.submit_and_complete_admin(|c_id, _| {
            NvmeCommand::create_io_submission_queue(
                c_id,
                q_id,
                sub_queue.get_addr(),
                (len - 1) as u16,
                q_id,
            )
        })?;

        self.q_id += 1;
        Ok(NvmeQueuePair {
            id: q_id,
            sub_queue,
            comp_queue,
        })
    }

    pub fn identify_namespace_list(&mut self, base: u32) -> Vec<u32> {
        self.submit_and_complete_admin(|c_id, addr| {
            NvmeCommand::identify_namespace_list(c_id, addr, base)
        });

        // TODO: idk bout this/don't hardcode len
        let data: &[u32] =
            // unsafe { std::slice::from_raw_parts(self.buffer.virt.as_ptr() as *const u32, 1024) };
            unsafe { std::slice::from_raw_parts(self.buffer.virt as *const u32, 1024) };

        data.iter()
            .copied()
            .take_while(|&id| id != 0)
            .collect::<Vec<u32>>()
    }

    pub fn identify_namespace(&mut self, id: u32) -> NvmeNamespace {
        self.submit_and_complete_admin(|c_id, addr| {
            NvmeCommand::identify_namespace(c_id, addr, id)
        });

        let namespace_data: IdentifyNamespaceData =
            unsafe { *(self.buffer.virt as *const IdentifyNamespaceData) };

        // let namespace_data = unsafe { *tmp_buff.virt };
        let size = namespace_data.nsze;
        let blocks = namespace_data.ncap;

        // figure out block size
        let flba_idx = (namespace_data.flbas & 0xF) as usize;
        let flba_data = (namespace_data.lba_format_support[flba_idx] >> 16) & 0xFF;
        let block_size = if !(9..32).contains(&flba_data) {
            0
        } else {
            1 << flba_data
        };

        // TODO: check metadata?
        println!("Namespace {id}, Size: {size}, Blocks: {blocks}, Block size: {block_size}");

        let namespace = NvmeNamespace {
            id,
            blocks,
            block_size,
        };
        self.namespaces.insert(id, namespace);
        namespace
    }

    // pass prp list?
    pub fn write(&mut self, data: &impl DmaSlice, mut lba: u64) -> Result<(), Box<dyn Error>> {
        let ns = *self.namespaces.get(&1).unwrap();

        for chunk in data.chunks(2 * 4096) {
            let blocks = (chunk.slice.len() as u64 + ns.block_size - 1) / ns.block_size;
            self.namespace_io(&ns, blocks, lba, chunk.phys_addr as u64, true)?;
            lba += blocks;
        }

        Ok(())
    }

    pub fn read(
        &mut self,
        dest: &impl DmaSlice,
        mut lba: u64,
    ) -> Result<(), Box<dyn Error>> {
        let ns = *self.namespaces.get(&1).unwrap();

        for chunk in dest.chunks(2 * 4096) {
            let blocks = (chunk.slice.len() as u64 + ns.block_size - 1) / ns.block_size;
            self.namespace_io(&ns, blocks, lba, chunk.phys_addr as u64, false)?;
            lba += blocks;
        }
        Ok(())
    }

    pub fn write_copied(&mut self, data: &[u8], mut lba: u64) -> Result<(), Box<dyn Error>> {
        let ns = *self.namespaces.get(&1).unwrap();

        // for chunk in data.chunks(128 * 4096) {
        for chunk in data.chunks(2 * 4096) {
            self.buffer[..chunk.len()].copy_from_slice(chunk);
            let blocks = (chunk.len() as u64 + ns.block_size - 1) / ns.block_size;
            self.namespace_io(&ns, blocks, lba, self.buffer.phys as u64, true)?;
            lba += blocks;
        }

        Ok(())
    }

    pub fn read_copied(
        &mut self,
        ns_id: u32,
        dest: &mut [u8],
        mut lba: u64,
    ) -> Result<(), Box<dyn Error>> {
        let ns = *self.namespaces.get(&ns_id).unwrap();

        // for chunk in dest.chunks_mut(128 * 4096) {
        for chunk in dest.chunks_mut(2 * 4096) {
            let blocks = (chunk.len() as u64 + ns.block_size - 1) / ns.block_size;
            self.namespace_io(&ns, blocks, lba, self.buffer.phys as u64, false)?;

            lba += blocks;
            chunk.copy_from_slice(&self.buffer[..chunk.len()]);
        }
        Ok(())
    }

    pub fn submit_io(
        &mut self,
        ns: &NvmeNamespace,
        addr: u64,
        blocks: u64,
        lba: u64,
        write: bool,
    ) -> Option<usize> {
        assert!(blocks > 0);
        assert!(blocks <= 0x1_0000);
        let q_id = 1;

        let bytes = blocks * ns.block_size;
        let ptr1 = if bytes <= 4096 {
            0
        } else if bytes <= 8192 {
            addr + 4096 // self.page_size
        } else {
            // TODOo: idk if correct
            let offset = (addr - self.prp_list.phys as u64) / 8;
            self.prp_list.phys as u64 + offset
        };

        let entry = if write {
            NvmeCommand::io_write(self.io_sq.tail as u16, ns.id, lba, blocks as u16 - 1, addr, ptr1)
        } else {
            NvmeCommand::io_read(self.io_sq.tail as u16, ns.id, lba, blocks as u16 - 1, addr, ptr1)
        };
        self.io_sq.submit_checked(entry)
    }

    pub fn complete_io(&mut self, step: u64) -> Option<u16> {
        let q_id = 1;

        let (tail, c_entry, _) = self.io_cq.complete_n(step as usize);
        self.write_reg_idx(NvmeArrayRegs::CQyHDBL, q_id as u16, tail as u32);

        let status = c_entry.status >> 1;
        if status != 0 {
            eprintln!(
                "Status: 0x{:x}, Status Code 0x{:x}, Status Code Type: 0x{:x}",
                status,
                status & 0xFF,
                (status >> 8) & 0x7
            );
            eprintln!("{:?}", c_entry);
            return None;
        }
        self.stats.completions += 1;
        Some(c_entry.sq_head)
    }

    pub fn batched_write(
        &mut self,
        ns_id: u32,
        data: &[u8],
        mut lba: u64,
        batch_len: u64,
    ) -> Result<(), Box<dyn Error>> {
        let ns = *self.namespaces.get(&ns_id).unwrap();
        let block_size = 512;
        let q_id = 1;

        for chunk in data.chunks(HUGE_PAGE_SIZE) {
            self.buffer[..chunk.len()].copy_from_slice(chunk);
            let tail = self.io_sq.tail;

            let batch_len = std::cmp::min(batch_len, chunk.len() as u64 / block_size);
            let batch_size = chunk.len() as u64 / batch_len;
            let blocks = batch_size / ns.block_size;

            for i in 0..batch_len {
                if let Some(tail) = self.submit_io(
                    &ns,
                    self.buffer.phys as u64 + i * batch_size,
                    blocks,
                    lba,
                    true,
                ) {
                    self.stats.submissions += 1;
                    self.write_reg_idx(NvmeArrayRegs::SQyTDBL, q_id as u16, tail as u32);
                } else {
                    eprintln!("tail: {tail}, batch_len: {batch_len}, batch_size: {batch_size}, blocks: {blocks}");
                }
                lba += blocks;
            }
            self.io_sq.head = self.complete_io(batch_len).unwrap() as usize;
        }

        Ok(())
    }

    pub fn batched_read(
        &mut self,
        ns_id: u32,
        data: &mut [u8],
        mut lba: u64,
        batch_len: u64,
    ) -> Result<(), Box<dyn Error>> {
        let ns = *self.namespaces.get(&ns_id).unwrap();
        let block_size = 512;
        let q_id = 1;

        for chunk in data.chunks_mut(HUGE_PAGE_SIZE) {
            let tail = self.io_sq.tail;

            let batch_len = std::cmp::min(batch_len, chunk.len() as u64 / block_size);
            let batch_size = chunk.len() as u64 / batch_len;
            let blocks = batch_size / ns.block_size;

            for i in 0..batch_len {
                if let Some(tail) = self.submit_io(
                    &ns,
                    self.buffer.phys as u64 + i * batch_size,
                    blocks,
                    lba,
                    false,
                ) {
                    self.stats.submissions += 1;
                    self.write_reg_idx(NvmeArrayRegs::SQyTDBL, q_id as u16, tail as u32);
                } else {
                    eprintln!("tail: {tail}, batch_len: {batch_len}, batch_size: {batch_size}, blocks: {blocks}");
                }
                lba += blocks;
            }
            self.io_sq.head = self.complete_io(batch_len).unwrap() as usize;
            chunk.copy_from_slice(&self.buffer[..chunk.len()]);
        }
        Ok(())
    }

    pub fn namespace_io(
        &mut self,
        ns: &NvmeNamespace,
        blocks: u64,
        lba: u64,
        addr: u64,
        write: bool,
    ) -> Result<(), Box<dyn Error>> {
        assert!(blocks > 0);
        assert!(blocks <= 0x1_0000);

        let q_id = 1;

        let bytes = blocks * ns.block_size;
        let ptr1 = if bytes <= 4096 {
            0
        } else if bytes <= 8192 {
            // self.buffer.phys as u64 + 4096 // self.page_size
            addr + 4096 // self.page_size
        } else {
            // self.prp_list.phys as u64
            eprintln!("tough luck");
            addr + 4096
        };

        let entry = if write {
            NvmeCommand::io_write(
                self.io_sq.tail as u16,
                ns.id,
                lba,
                blocks as u16 - 1,
                addr,
                ptr1,
            )
        } else {
            NvmeCommand::io_read(
                self.io_sq.tail as u16,
                ns.id,
                lba,
                blocks as u16 - 1,
                addr,
                ptr1,
            )
        };

        let tail = self.io_sq.submit(entry);
        self.stats.submissions += 1;

        self.write_reg_idx(NvmeArrayRegs::SQyTDBL, q_id as u16, tail as u32);
        self.io_sq.head = self.complete_io(1).unwrap() as usize;
        Ok(())
    }


    pub fn submit_and_complete_admin<F: FnOnce(u16, usize) -> NvmeCommand>(
        &mut self,
        cmd_init: F,
    ) -> Result<NvmeCompletion, Box<dyn Error>> {
        let cid = self.admin_sq.tail;
        let tail = self.admin_sq.submit(cmd_init(cid as u16, self.buffer.phys));
        self.write_reg_idx(NvmeArrayRegs::SQyTDBL, 0, tail as u32);

        let (head, entry, _) = self.admin_cq.complete_spin();
        self.write_reg_idx(NvmeArrayRegs::CQyHDBL, 0, head as u32);
        let status = entry.status >> 1;
        if status != 0 {
            eprintln!(
                "Status: 0x{:x}, Status Code 0x{:x}, Status Code Type: 0x{:x}",
                status,
                status & 0xFF,
                (status >> 8) & 0x7
            );
            return Err("Requesting i/o completion queue failed".into());
        }
        Ok(entry)
    }

    /// Sets Queue `qid` Tail Doorbell to `val`
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
