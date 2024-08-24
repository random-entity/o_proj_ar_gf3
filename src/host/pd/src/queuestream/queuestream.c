#include <stdlib.h>

#include "m_pd.h"

static t_class* queuestream_class;

typedef struct _queuestream {
  t_object o;
  t_float* data;
  uint16_t size;
} t_queuestream;

void queuestream_bangin1(t_queuestream* x) {
  t_atom list[x->size];
  for (uint16_t i = 0; i < x->size; i++) SETFLOAT(&list[i], x->data[i]);
  outlet_list(x->o.ob_outlet, &s_list, x->size, list);
}

void queuestream_floatin1(t_queuestream* x, t_float f) {
  if (x->size > 1) {
    for (uint16_t i = 0; i < x->size - 1; i++) x->data[i] = x->data[i + 1];
  }
  x->data[x->size - 1] = f;
  queuestream_bangin1(x);
}

void* queuestream_new(t_floatarg f) {
  t_queuestream* x = (t_queuestream*)pd_new(queuestream_class);

  x->size = (int)f > 0 ? (int)f : 10;
  x->data = (t_float*)malloc(x->size * sizeof(t_float));
  for (uint16_t i = 0; i < x->size; i++) x->data[i] = 0;
  outlet_new(&x->o, &s_list);
  return (void*)x;
}

void queuestream_free(t_queuestream* x) { free(x->data); }

void queuestream_setup() {
  queuestream_class =
      class_new(gensym("queuestream"), (t_newmethod)queuestream_new,
                (t_method)queuestream_free, sizeof(t_queuestream),
                CLASS_DEFAULT, A_DEFFLOAT, 0);

  class_addbang(queuestream_class, queuestream_bangin1);
  class_addfloat(queuestream_class, queuestream_floatin1);
}
