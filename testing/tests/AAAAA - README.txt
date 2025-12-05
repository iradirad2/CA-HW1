let i=4; while((i<150)); do ./bp_main tests/example${i}.trc > tests/your_output_for_example${i}.out; let i++; done

let i=4; while((i<150)); do diff -s -q tests/example${i}.out tests/your_output_for_example${i}.out; let i++; done

let i=4; while((i<150)); do ./bp_main tests/n_example${i}.trc > tests/your_output_for_n_example${i}.out; let i++; done

let i=4; while((i<150)); do diff -s -q tests/n_example${i}.out tests/your_output_for_n_example${i}.out; let i++; done

let i=4; while((i<150)); do ./bp_main tests/small_btb_example${i}.trc > tests/your_output_for_small_btb_example${i}.out; let i++; done

let i=4; while((i<150)); do diff -s -q tests/small_btb_example${i}.out tests/your_output_for_small_btb_example${i}.out; let i++; done

let i=1; while((i<21)); do ./bp_main tests/segel/example${i}.trc > tests/segel/your_segel_example${i}.out; let i++; done

let i=1; while((i<21)); do diff -s -q tests/segel/example${i}.out tests/segel/your_segel_example${i}.out; let i++; done

let i=1; while((i<4)); do ./bp_main input_examples/example${i}.trc > input_examples/your_output_example${i}.out; let i++; done

let i=1; while((i<4)); do diff -s -q input_examples/your_output_example${i}.out input_examples/example${i}.out; let i++; done