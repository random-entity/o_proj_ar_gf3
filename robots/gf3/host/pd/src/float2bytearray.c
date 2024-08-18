#include "m_pd.h"

static t_class* float2bytearray_class;

typedef struct _float2bytearray {
  t_object x_obj;
  t_float input_value;
  t_outlet* out1;
  t_outlet* out2;
  t_outlet* out3;
  t_outlet* out4;
} t_float2bytearray;

void float2bytearray_bang(t_float2bytearray* x) {
  union {
    float f;
    unsigned char bytes[4];
  } float_union;

  float_union.f = x->input_value;

  outlet_float(x->out1, float_union.bytes[0]);
  outlet_float(x->out2, float_union.bytes[1]);
  outlet_float(x->out3, float_union.bytes[2]);
  outlet_float(x->out4, float_union.bytes[3]);
}

void float2bytearray_float(t_float2bytearray* x, t_float f) {
  x->input_value = f;
}

void* float2bytearray_new() {
  t_float2bytearray* x = (t_float2bytearray*)pd_new(float2bytearray_class);

  // Initialize float value
  x->input_value = 0.0;

  // Create inlets: inlet 1 (bang) and inlet 2 (float)
  inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_float, gensym("float"));

  // Create outlets
  x->out1 = outlet_new(&x->x_obj, &s_float);
  x->out2 = outlet_new(&x->x_obj, &s_float);
  x->out3 = outlet_new(&x->x_obj, &s_float);
  x->out4 = outlet_new(&x->x_obj, &s_float);

  return (void*)x;
}

void float2bytearray_free(t_float2bytearray* x) {}

void float2bytearray_setup(void) {
  float2bytearray_class =
      class_new(gensym("float2bytearray"), (t_newmethod)float2bytearray_new,
                (t_method)float2bytearray_free, sizeof(t_float2bytearray),
                CLASS_DEFAULT, 0);

  class_addbang(float2bytearray_class, float2bytearray_bang);
  class_addfloat(float2bytearray_class, float2bytearray_float);
}
