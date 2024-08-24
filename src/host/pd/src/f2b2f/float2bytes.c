/// Number input from the only inlet
/// will be stored in value.f.
/// Number or bang input on the inlet will output union bytes.

#include "m_pd.h"

static t_class* float2bytes_class;

typedef struct _float2bytes {
  t_object o;
  union {
    float f;
    uint8_t bytes[4];
  } value;
  t_outlet* out[4];
} t_float2bytes;

void float2bytes_bangin1(t_float2bytes* x) {
  // Output from right to left!
  for (uint8_t i = 3; i != 255; i--) outlet_float(x->out[i], x->value.bytes[i]);
}

void float2bytes_floatin1(t_float2bytes* x, t_float f) {
  x->value.f = f;
  float2bytes_bangin1(x);
}

void* float2bytes_new() {
  t_float2bytes* x = (t_float2bytes*)pd_new(float2bytes_class);
  x->value.f = 0.0f;
  for (uint8_t i = 0; i < 4; i++) x->out[i] = outlet_new(&x->o, &s_float);
  return (void*)x;
}

void float2bytes_free(t_float2bytes* x) {
  for (uint8_t i = 0; i < 4; i++) outlet_free(x->out[i]);
}

void float2bytes_setup() {
  float2bytes_class = class_new(
      gensym("float2bytes"), (t_newmethod)float2bytes_new,
      (t_method)float2bytes_free, sizeof(t_float2bytes), CLASS_DEFAULT, 0);

  class_addfloat(float2bytes_class, float2bytes_floatin1);
  class_addbang(float2bytes_class, float2bytes_bangin1);
}
