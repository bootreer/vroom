use std::{env, process, thread};
use std::error::Error;
use std::sync::{Arc, Mutex};
use vroom::memory::*;
use vroom::{NvmeDevice, QUEUE_LENGTH};
use rand::{thread_rng, Rng};

pub fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args();
    args.next();

    let pci_addr = match args.next() {
        Some(arg) => arg,
        None => {
            eprintln!("Usage: cargo run --example init <pci bus id>");
            process::exit(1);
        }
    };

    let nvme = vroom::init(&pci_addr)?;
    // nvme.create_io_queue_pair(QUEUE_LENGTH)?;
    let _ = qd32(nvme)?;

    /*
    // Testing stuff
    let blocks = 8;
    let bytes = 512 * blocks;
    let ns_blocks = nvme.namespaces.get(&1).unwrap().blocks - blocks - 1;

    let mut buffer: Dma<u8> = Dma::allocate(HUGE_PAGE_SIZE, true)?;

    let n = 1_000_000;
    let mut read = std::time::Duration::new(0, 0);
    let mut write = std::time::Duration::new(0, 0);

    let mut rng = thread_rng();
    let seq = &(0..n).map(|_| rng.gen_range(0..ns_blocks as u64)).collect::<Vec<u64>>()[..];
    for &lba in seq {
        /* 
        let lba = rng.gen_range(0..ns_blocks);
        let rand_block = &(0..bytes).map(|_| rand::random::<u8>()).collect::<Vec<_>>()[..];

        buffer[..rand_block.len()].copy_from_slice(rand_block);

        // write
        let before = std::time::Instant::now();
        nvme.write(&buffer.slice(0..bytes as usize), lba)?;
        write += before.elapsed();

        buffer[..rand_block.len()].fill_with(Default::default);
        */
        let before = std::time::Instant::now();
        nvme.read(&buffer.slice(0..bytes as usize), lba)?;
        read += before.elapsed();

        // assert_eq!(&buffer[0..rand_block.len()], rand_block);
        // lba += blocks as u64;
    }

    println!("total completions: {}", nvme.stats.completions);
    println!("total submissions: {}", nvme.stats.submissions);
    println!(
        "read time: {:?}; write time: {:?}; total: {:?}",
        read,
        write,
        read + write
    );
    */

    Ok(())
}

#[allow(unused)]
fn qd32(mut nvme: NvmeDevice) -> Result<NvmeDevice, Box<dyn Error>> {
    let blocks = 8;
    let bytes = 512 * blocks;
    let ns_blocks = Arc::new(nvme.namespaces.get(&1).unwrap().blocks - blocks - 1);

    let nvme = Arc::new(Mutex::new(nvme));
    let mut threads = Vec::new();

    let before = std::time::Instant::now();
    for _ in 0..4 {
        let max_lba = ns_blocks.clone();
        let mut nvme = Arc::clone(&nvme);

        let handle = thread::spawn(move || -> std::time::Duration {
            let mut rng = rand::thread_rng();
            let seq = &(0..1_000_000).map(|_| rng.gen_range(0..*max_lba)).collect::<Vec<u64>>()[..];

            let blocks = 8;
            let bytes = 512 * blocks;

            let mut total= std::time::Duration::ZERO;
            let mut buffer: Dma<u8> = Dma::allocate(HUGE_PAGE_SIZE, true).unwrap();

            let mut qpair = nvme.lock().unwrap().create_io_queue_pair(32).unwrap();

            // TODO
            for lba in seq.chunks(32) {
                // let rand_block = &(0..(32 * bytes)).map(|_| rand::random::<u8>()).collect::<Vec<_>>()[..];
                // buffer[..rand_block.len()].copy_from_slice(rand_block);

                let before = std::time::Instant::now();
                for (idx, &lba) in lba.iter().enumerate() { 
                    qpair.submit_io(&buffer.slice((idx * bytes)..(idx + 1) * bytes as usize), lba, false);
                }
                while !qpair.sub_queue.is_empty() {
                    if let Some(head) = qpair.complete_io(1) {
                        qpair.sub_queue.head = head as usize;
                    } else {
                        eprintln!("shit"); 
                        continue;
                    }
                }
                total += before.elapsed();

            }
            assert!(qpair.sub_queue.is_empty());

            total
        });
        threads.push(handle);
    }

    let total = threads.into_iter().fold(0., |acc, thread| {
        let res = thread
            .join()
            .expect("The thread creation or execution failed!");
        println!("elapsed: {:?}", res);
        acc + 1_000_000. / res.as_secs_f64()
    });
    println!("total iops: {:?}", total);

    // lol
    match Arc::try_unwrap(nvme) {
        Ok(mutex) => {
            match mutex.into_inner() {
                Ok(t) => Ok(t),
                Err(e) => Err(e.into()),
            }
        }
        Err(arc_mutex_t_again) => {
            Err("Arc::try_unwrap failed, not the last reference.".into())
        }
    }
}
