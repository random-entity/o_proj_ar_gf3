#include "m_pd.h"

static t_class* bytearray2float_class;

typedef struct _bytearray2float {
  t_object x_obj;
  unsigned char bytes[4];
  t_outlet* out;
} t_bytearray2float;

void bytearray2float_bang(t_bytearray2float* x) {
  union {
    float f;
    unsigned char bytes[4];
  } float_union;

  float_union.bytes[0] = x->bytes[0];
  float_union.bytes[1] = x->bytes[1];
  float_union.bytes[2] = x->bytes[2];
  float_union.bytes[3] = x->bytes[3];

  outlet_float(x->out, float_union.f);
}

void bytearray2float_set_byte1(t_bytearray2float* x, t_float f) {
  x->bytes[0] = (unsigned char)f;
}

void bytearray2float_set_byte2(t_bytearray2float* x, t_float f) {
  x->bytes[1] = (unsigned char)f;
}

void bytearray2float_set_byte3(t_bytearray2float* x, t_float f) {
  x->bytes[2] = (unsigned char)f;
}

void bytearray2float_set_byte4(t_bytearray2float* x, t_float f) {
  x->bytes[3] = (unsigned char)f;
}

void* bytearray2float_new(void) {
  t_bytearray2float* x = (t_bytearray2float*)pd_new(bytearray2float_class);

  // Initialize bytes
  x->bytes[0] = 0;
  x->bytes[1] = 0;
  x->bytes[2] = 0;
  x->bytes[3] = 0;

  // Create inlets for the 4 bytes
  inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_float, gensym("byte1"));
  inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_float, gensym("byte2"));
  inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_float, gensym("byte3"));
  inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_float, gensym("byte4"));

  // Create outlet for the resulting float
  x->out = outlet_new(&x->x_obj, &s_float);

  return (void*)x;
}

void bytearray2float_free(t_bytearray2float* x) {}

void bytearray2float_setup(void) {
  bytearray2float_class =
      class_new(gensym("bytearray2float"), (t_newmethod)bytearray2float_new,
                (t_method)bytearray2float_free, sizeof(t_bytearray2float),
                CLASS_DEFAULT, 0);

  class_addbang(bytearray2float_class, bytearray2float_bang);
  class_addmethod(bytearray2float_class, (t_method)bytearray2float_set_byte1,
                  gensym("byte1"), A_FLOAT, 0);
  class_addmethod(bytearray2float_class, (t_method)bytearray2float_set_byte2,
                  gensym("byte2"), A_FLOAT, 0);
  class_addmethod(bytearray2float_class, (t_method)bytearray2float_set_byte3,
                  gensym("byte3"), A_FLOAT, 0);
  class_addmethod(bytearray2float_class, (t_method)bytearray2float_set_byte4,
                  gensym("byte4"), A_FLOAT, 0);
}
