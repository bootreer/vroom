use std::env;
use std::process;
use vroom::QUEUE_LENGTH;
use vroom::HUGE_PAGE_SIZE;
use vroom::memory::Dma;

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args();
    args.next();

    let pci_addr = match args.next() {
        Some(arg) => arg,
        None => {
            eprintln!("Usage: cargo run --example init <pci bus id>");
            process::exit(1);
        }
    };

    let mut nvme = vroom::init(&pci_addr)?;
    nvme.create_io_queue_pair(QUEUE_LENGTH)?;

    // Testing stuff
    let n = 10;
    let n2 = 1000;
    let blocks = 8;
    let mut buffer: Dma<[u8; HUGE_PAGE_SIZE]> = Dma::allocate(HUGE_PAGE_SIZE, true)?;

    let mut read = std::time::Duration::new(0, 0);
    let mut write = std::time::Duration::new(0, 0);
    //  let mut write_batched = std::time::Duration::new(0, 0);
    //  let mut read_batched = std::time::Duration::new(0, 0);

    let mut rng = rand::thread_rng();
    use rand::seq::SliceRandom;

    let mut seq: Vec<u64> = Vec::from_iter(0..n);

    for _ in 0..n2 {
        seq.shuffle(&mut rng);
        let lba = 0;
        for i in &seq {
            let rand_block = &(0..(512 * blocks))
                .map(|_| rand::random::<u8>())
                .collect::<Vec<_>>()[..];

            buffer[..rand_block.len()].copy_from_slice(rand_block);

            // write
            let before = std::time::Instant::now();
            nvme.write_raw(rand_block, lba + (*i * blocks as u64), buffer.phys as u64)?;
            write += before.elapsed();

            //  let before = Instant::now();
            //  nvme.batched_write(1, rand_block, lba, 256)?;
            //  write_batched += before.elapsed();

            // read
            //  let before = Instant::now();
            //  nvme.batched_read(1, &mut read_bbuf[..], lba, 256)?;
            //  read_batched += before.elapsed();

            buffer[..rand_block.len()].fill_with(Default::default);
            let before = std::time::Instant::now();
            nvme.read(
                1,
                &buffer[..rand_block.len()],
                lba + (*i * blocks as u64),
                buffer.phys as u64,
            )?;
            read += before.elapsed();

            assert_eq!(&buffer[..rand_block.len()], rand_block);
            //  lba += blocks as u64;
        }
    }

    println!("total completions: {}", nvme.stats.completions);
    println!("total submissions: {}", nvme.stats.submissions);
    println!(
        "read time: {:?}; write time: {:?}; total: {:?}",
        read,
        write,
        read + write
    );
    Ok(())
}
