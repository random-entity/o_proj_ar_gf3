* md  = Mode
* tva = MaxTrqVelAcc

* so  = SajOutput
* do  = DjAvgDiffOutputs

* sj  = md + tva + so
* dj  = md + tva + do

md tva zero so do
1  1   1         |  global
                 |  |
                 |  |------------
                 |  |           |
1  1        1    |  sajs        |
1  1           1 |  |           djs
                 |  |           |
                 |  |------     |--------------------------
                 |  |     |     |            |            |
                 |  shzs  wrs   shxys        elbs         neck
                 |  l, r  l, r  l,    r      l,    r      l, r
                 |              ll,lr,rl,rr  ll,lr,rl,rr