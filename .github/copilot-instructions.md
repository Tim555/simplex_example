# Project Instructions

## Project Context

This project is designed to understand the simplex method for LP (Linear Programming). It provides an implementation of the simplex algorithm along with examples, tests and benchmarks to facilitate learning and experimentation.

The project is written in C++, using the Eigen library for linear algebra operations. Google test is used for unit testing. CMake is used as the build system.  Clang-tidy is used for static code analysis. Clang-format is used for code formatting.


## Development Workflow

- Follow the existing project structure and conventions.
- Keep function names short and descriptive. Use verbs to indicate the action performed by the function.
- Keep changes focused on the requested behavior.
- Prefer small, readable implementations over unnecessary abstractions.
- Update documentation when behavior or setup changes.

## Validation

- Configure with `cmake -S . -B build`.
- Build with `cmake --build build`.
- Run tests with `ctest --test-dir build --output-on-failure`.
- Format changed C++ files with `clang-format`.
- Run `cmake --build build --target format` to format project C++ files.
- Enable linting with `cmake -S . -B build-tidy -DENABLE_CLANG_TIDY=ON`, then build with `cmake --build build-tidy`.
- Add or update focused tests for changed behavior.
- Report any validation command that could not be run and why.

## Code Style

- Match the surrounding code style and naming conventions.
- Avoid unrelated refactors or generated-file changes.
- Keep public interfaces stable unless the task explicitly requires a breaking change.
- Check that clang-format and clang-tidy have been run on changed files.

## Project-Specific Notes

- Production sources live in `src/`, tests live in `test/`, and built executables are written to `bin/`.
- Use C++17 and CMake for the build.
- Use GoogleTest for unit tests.
- Keep `.clang-format` and `.clang-tidy` in sync with the project conventions.
