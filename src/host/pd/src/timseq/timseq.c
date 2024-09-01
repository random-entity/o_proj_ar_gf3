#include <stdbool.h>
#include <stdlib.h>

#include "m_pd.h"

static t_class* timseq_class;

typedef struct _timseq {
  t_object o;
  t_float* data;
  t_float* intervals;
  uint16_t size;
  uint16_t cur_idx;
  t_clock* clk;
  bool running;
  t_outlet* out_data;
  t_outlet* out_stopbang;
  t_outlet* out_finishbang;
} t_timseq;

void timseq_start(t_timseq* x) {
  if (x->running) {
    post("timseq: already started; Stop to restart");
  } else {
    x->running = true;
    x->cur_idx = 0;
    timseq_tick(x);
  }
}

void timseq_stop(t_timseq* x) {
  if (x->running) {
    x->running = false;
  }
  outlet_bang(x->out_stopbang);
}

void timseq_tick(t_timseq* x) {
  if (x->running) {
    if (x->cur_idx < x->size) {
      outlet_float(x->out_data, x->data[x->cur_idx]);
      x->cur_idx++;
      clock_delay(x->clk, x->intervals[x->cur_idx - 1]);
    } else {
      outlet_bang(x->out_finishbang);
      x->running = false;
    }
  }
}

void timseq_listin(t_timseq* x, t_symbol* s, int argc, t_atom* argv) {
  if (argc < 2) {
    post("timseq: expected 2 or more elements in list");
    return;
  }
  if (argc % 2 != 0) {
    post("timseq: expected an even number of elements in list");
    return;
  }

  x->size = (uint16_t)(argc / 2);

  if (x->data) freebytes(x->data, x->size * sizeof(t_float));
  x->data = (t_float*)getbytes(x->size * sizeof(t_float));

  if (x->intervals) freebytes(x->intervals, x->size * sizeof(t_float));
  x->intervals = (t_float*)getbytes(x->size * sizeof(t_float));

  for (uint16_t i = 0; i < x->size; i++) {
    x->data[i] = atom_getfloat(argv + 2 * i);
    x->intervals[i] = 1000.0 * atom_getfloat(argv + 2 * i + 1);
  }
}

void* timseq_new() {
  t_timseq* x = (t_timseq*)pd_new(timseq_class);

  x->data = NULL;
  x->intervals = NULL;
  x->size = 0;
  x->cur_idx = 0;
  x->clk = clock_new(x, (t_method)timseq_tick);
  x->out_data = outlet_new(&x->o, &s_float);
  x->out_stopbang = outlet_new(&x->o, &s_bang);
  x->out_finishbang = outlet_new(&x->o, &s_bang);

  /* Inlet 1 (the default inlet) which does not have to be
     explicitly created by inlet_new, will handle start bang. */
  // Inlet 2
  inlet_new(&x->o, &x->o.ob_pd, &s_bang, gensym("stopbang"));
  // Inlet 3
  inlet_new(&x->o, &x->o.ob_pd, &s_list, gensym("listin"));

  return (void*)x;
}

void timseq_free(t_timseq* x) {
  if (x->data) freebytes(x->data, x->size * sizeof(t_float));
  if (x->intervals) freebytes(x->intervals, x->size * sizeof(t_float));
  clock_free(x->clk);
}

void timseq_setup() {
  timseq_class =
      class_new(gensym("timseq"), (t_newmethod)timseq_new,
                (t_method)timseq_free, sizeof(t_timseq), CLASS_DEFAULT, 0);

  // Inlet 1
  class_addbang(timseq_class, timseq_start);
  // Inlet 2
  class_addmethod(timseq_class, (t_method)timseq_stop, gensym("stopbang"), 0);
  // Inlet 3
  class_addmethod(timseq_class, (t_method)timseq_listin, gensym("listin"),
                  A_GIMME, 0);
}
