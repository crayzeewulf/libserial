let g:ale_cpp_clang_options="-std=c++14 -Wall -Isrc -Weffc++"
let g:ale_cpp_clangtidy_options=g:ale_cpp_clang_options
let g:ale_cpp_clangtidy_checks=['*', '-fuchsia-default-arguments', '-google-runtime-int', '-llvm-header-guard']
let g:ale_cpp_gcc_options=g:ale_cpp_clang_options
