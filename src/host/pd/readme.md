* md   = Mode
* mtva = MaxTrqVelAcc

* so   = SajOutput
* do   = DjAvgDiffOutputs

* sj   = md + mtva + so
* dj   = md + mtva + do

md mtva zero so do
1  1    1         |  global
                  |  |
                  |  |------------
                  |  |           |
1  1         1    |  sajs        |
1  1            1 |  |           djs
                  |  |           |
                  |  |------     |--------------------------
                  |  |     |     |            |            |
                  |  shzs  wrs   shxys        elbs         neck
                  |  l, r  l, r  l,    r      l,    r      l, r
                  |              ll,lr,rl,rr  ll,lr,rl,rr