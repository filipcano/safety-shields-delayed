import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import rc



def filtereven(a):
    res = []
    for i in range(len(a)):
        if i%2 == 0:
            res.append(a[i])
    return res

plt.rcParams.update({'font.size':28})
rc('font', **{'family': 'serif', 'serif': ['Times New Roman']})
rc('text', usetex=True)

with open('exp2.txt', 'r') as file:
    data = file.read()

cols = ['delta', 'pre_shield','safest', 'most_controllable', 'pop_sfty', 'pop_ctrl']
df = pd.DataFrame(columns=cols)
delta = 0
trivial = 0
safest = 0
controllable = 0
pop_sfty = 0
pop_ctrl = 0

for line in data.split('\n')[1:]:
    if len(line) < 5:
        continue
    if 'delta' in line:
        delta = int(line.split('delta = ')[-1].split(' in')[0])
    # if (delta == 0):
        # continue
    df.loc[delta,'delta'] = delta
    newtime = float(line.split('in ')[-1].split(' seconds')[0])
    if 'safety vector' in line:
        pop_sfty = newtime
    elif 'controllability vector' in line:
        pop_ctrl = newtime
    elif 'safety' in line:
        df.loc[delta, 'safest'] = newtime
    elif 'controllability' in line:
        df.loc[delta, 'most_controllable'] = newtime
    else :
        df.loc[delta, 'pre_shield'] = newtime

original_algorithm_p1 = 195.03+216.746+464.097+920.745+1436.55
original_algorithm_p2 = 1966.26 + 2194.29+4165.77+7349.38+11483.8

df['original'] = [2194, 4165, 7349, 11483, np.inf, np.inf, np.inf]

for i in df.index[1:]:
    df.loc[i,'pre_shield'] = df.loc[i,'pre_shield'] + df.loc[i-1,'pre_shield']
df.safest = df.pre_shield + df.safest + pop_sfty
df.most_controllable = df.most_controllable + pop_ctrl + df.loc[df.index[-1],'pre_shield']


markersize = 18
linewidth = 3
figfactor = 0.65
fig, ax1 = plt.subplots(figsize=(21*figfactor,10*figfactor))
ax1.plot(filtereven((df.delta/2).astype('int')), filtereven(df.pre_shield), color = 'r', marker = 'o', alpha = 0.6, markersize=markersize, linewidth=linewidth)
ax1.plot(filtereven((df.delta/2).astype('int')), filtereven(df.safest), color = 'g', marker = '^', alpha = 0.6, markersize=markersize, linewidth=linewidth)
ax1.plot(filtereven((df.delta/2).astype('int')), filtereven(df.most_controllable), color = 'b', marker = 's', alpha = 0.6, markersize=markersize, linewidth=linewidth)
ax1.plot(filtereven((df.delta/2).astype('int')), filtereven(df.original), color = 'orange', marker = 'X', alpha = 0.6, markersize=markersize, linewidth=linewidth)
ax1.set_xlabel('delay')
ax1.set_ylabel('time (s)')
ax1.set_yscale('log', base=10)
ax1.set_xlim(-0.2,filtereven((df.delta/2).astype('int'))[-1]+1)
for spine in ['right', 'top']:
    ax1.spines[spine].set_visible(False)
ax1.set_xticks(filtereven((df.delta/2).astype('int')))
ax1.legend(['Max.-Perm. Strat (Us)', 'Robust-Safety', 'Delay-Resilient', 'Max.-Perm. Strat (Chen et al.)'])
plt.savefig('exp2.pdf', bbox_inches='tight')
plt.show()
