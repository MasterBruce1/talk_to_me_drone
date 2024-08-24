import numpy as np

#.tflite
with open('B:/jupyter_notebook/821_/combined_model_int8.tflite', 'rb') as f:
    tflite_model = f.read()

c_array = ', '.join([f'0x{b:02x}' for b in tflite_model])

# .h 
with open('B:/jupyter_notebook/821_/model_combined_tflite.h', 'w') as f:
    f.write('#ifndef COMBINED_MODEL_INT8_TFLITE_H_\n')
    f.write('#define COMBINED_MODEL_INT8_TFLITE_H_\n\n')
    f.write('const unsigned char combined_model_int8_tflite[] = {\n')
    f.write(c_array)
    f.write('\n};\n\n')
    f.write('const int combined_model_int8_tflite_len = {};\n'.format(len(tflite_model)))
    f.write('#endif  // COMBINED_MODEL_INT8_TFLITE_H_\n')

