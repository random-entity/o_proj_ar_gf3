/// Number input from inlets 1, 2, 3, 4
/// will be stored in bytes[0, 1, 2, 3].
/// Number or bang input on inlet 1 will output union float.

#include "m_pd.h"

static t_class* bytes2float_class;

typedef struct _bytes2float {
  t_object o;
  union {
    float f;
    uint8_t bytes[4];
  } value;
  t_outlet* out;
} t_bytes2float;

void bytes2float_bangin1(t_bytes2float* x) { outlet_float(x->out, x->value.f); }

void bytes2float_floatin1(t_bytes2float* x, t_float f) {
  x->value.bytes[0] = (uint8_t)f;
  bytes2float_bangin1(x);
}

void bytes2float_floatin2(t_bytes2float* x, t_float f) {
  x->value.bytes[1] = (uint8_t)f;
}

void bytes2float_floatin3(t_bytes2float* x, t_float f) {
  x->value.bytes[2] = (uint8_t)f;
}

void bytes2float_floatin4(t_bytes2float* x, t_float f) {
  x->value.bytes[3] = (uint8_t)f;
}

void* bytes2float_new() {
  t_bytes2float* x = (t_bytes2float*)pd_new(bytes2float_class);

  x->value.f = 0.0f;

  /* Inlet 1 (the default inlet) which does not have to be
     explicitly created by inlet_new, will handle byte 0 */
  // Inlet 2
  inlet_new(&x->o, &x->o.ob_pd, &s_float, gensym("in2"));
  // Inlet 3
  inlet_new(&x->o, &x->o.ob_pd, &s_float, gensym("in3"));
  // Inlet 4
  inlet_new(&x->o, &x->o.ob_pd, &s_float, gensym("in4"));

  x->out = outlet_new(&x->o, &s_float);

  return (void*)x;
}

void bytes2float_free(t_bytes2float* x) { outlet_free(x->out); }

void bytes2float_setup() {
  bytes2float_class = class_new(
      gensym("bytes2float"), (t_newmethod)bytes2float_new,
      (t_method)bytes2float_free, sizeof(t_bytes2float), CLASS_DEFAULT, 0);

  // Inlet 1
  class_addbang(bytes2float_class, bytes2float_bangin1);
  class_addfloat(bytes2float_class, bytes2float_floatin1);
  // Inlet 2
  class_addmethod(bytes2float_class, (t_method)bytes2float_floatin2,
                  gensym("in2"), A_FLOAT, 0);
  // Inlet 3
  class_addmethod(bytes2float_class, (t_method)bytes2float_floatin3,
                  gensym("in3"), A_FLOAT, 0);
  // Inlet 4
  class_addmethod(bytes2float_class, (t_method)bytes2float_floatin4,
                  gensym("in4"), A_FLOAT, 0);
}
