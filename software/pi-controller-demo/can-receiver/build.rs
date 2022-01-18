extern crate cc;

fn main()
{
    cc::Build::new().file("src/linux_utils.c").compile("linux_utils.a");
}