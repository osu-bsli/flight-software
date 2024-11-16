fn main() {
    /*
     * TODO: MAVLink handling
     */

    /*
     * Generate Rust-to-C bindings (allow C to call to Rust code)
     */

    cbindgen::Builder::new()
        .with_src("Core/Src/main.rs")
        .with_language(cbindgen::Language::C)
        .with_cpp_compat(true)
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file("Core/Inc/rust_bindings.h");

    /*
     * Invoke CMake for STM32 C code.
     */

    let dst: std::path::PathBuf = cmake::Config::new(".").generator("Ninja").build();

    /*
     * Link the C code static library.
     */

    println!("cargo::rustc-link-search=native={}/lib", dst.display());
    println!("cargo::rustc-link-lib=flight-software-c");

    /*
     * Linker arguments. (copy-pasted from cmake/gcc-arm-none-eabi.cmake)
     */

    let pkg_name = std::env::var("CARGO_PKG_NAME").unwrap();

    let linker_args = [
        "-mcpu=cortex-m7",
        "-mfpu=fpv5-d16",
        "-mfloat-abi=hard",
        &format!("-Wl,-Map={pkg_name}.map"),
        "-TSTM32H753IITx_FLASH.ld",
        "--specs=nano.specs",
        "-Wl,--gc-sections",
        "-Wl,--start-group",
        "-lc",
        "-lm",
        "-Wl,--end-group",
        "-Wl,--print-memory-usage",
    ];
    
    for a in linker_args {
        println!("cargo::rustc-link-arg={a}");
    }

    /* Copy compile_commands.json to root directory for clangd VSCode extension */
    let mut compile_commands_path = dst.clone();
    compile_commands_path.push("build");
    compile_commands_path.push("compile_commands.json");
    std::fs::copy(compile_commands_path, "compile_commands.json").unwrap();
}
