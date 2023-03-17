import pandas as pd
import numpy as np
from matplotlib import rc
from matplotlib import pyplot as plt

plt.rcParams.update({'font.size': 20})
rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
rc('text', usetex=True)

with open('exp1.txt', 'r') as file:
    data = file.read()

cols = ['n', 'pre-shield','safest', 'most_controllable', 'pop_sfty', 'pop_ctrl']
df = pd.DataFrame(columns=cols)
n = 0
trivial = 0
safest = 0
controllable = 0
pop_sfty = 0
pop_ctrl = 0
n =1

for line in data.split('\n')[1:]:
    if line[0:6] == 'inputs' or line == '':
        df = df.append({'n': n, 'pre-shield': trivial, 'safest': safest, 'most_controllable':controllable,
                        'pop_sfty':pop_sfty, 'pop_ctrl':pop_ctrl}, ignore_index = True)
        if line == '':
            break
        n = int(line[11])
        trivial = 0
        safest = 0
        controllable = 0
        continue
    if (len(line) < 6):
        continue
    newdelta = float(line.split('in ')[-1].split(' seconds')[0])
    if 'safety vector' in line:
        pop_sfty = newdelta
    elif 'controllability vector' in line:
        pop_ctrl = newdelta
    elif 'safety' in line:
        safest = newdelta
    elif 'controllability' in line:
        controllable = newdelta
    else :
        trivial += newdelta

original_algorithm_p1 = 195.03+216.746+464.097+920.745+1436.55
original_algorithm_p2 = 1966.26 + 2194.29+4165.77+7349.38+11483.8

df['original'] = [original_algorithm_p1, original_algorithm_p2, np.inf, np.inf, np.inf, np.inf, np.inf]

df

for i in df.index[1:]:
    df.loc[i,'pre-shield'] = df.loc[i,'pre-shield'] + df.loc[i-1,'pre-shield']
df.safest = df.safest + df['pre-shield'] + df.pop_sfty
df.most_controllable = df.most_controllable + df['pre-shield'] + df.pop_ctrl

figfactor = 0.5
fig, ax1 = plt.subplots(figsize=(15*figfactor,10*figfactor))
ax1.plot(df.n.astype('int'), df['pre-shield'], color = 'r', marker = 'o', alpha = 0.6)
ax1.plot(df.n.astype('int'), df.safest, color = 'g', marker = '^')
ax1.plot(df.n.astype('int'), df.most_controllable, color = 'b', marker = 's')
ax1.plot(df.n.astype('int'), df.original, color = 'orange', marker = '*')
ax1.set_xlabel('n')
ax1.set_ylabel('time (s)')
ax1.set_yscale('log', base=10)
ax1.set_xticks(df.n.astype('int'))
ax1.legend(['Max.-Perm. Strat (Us)', 'Robust-Safety', 'Delay-Resilient', 'Max.-Perm. Strat (Chen et al.)'], loc='upper right')
# plt.tight_layout()
plt.savefig('exp1.pdf', bbox_inches='tight')
plt.show()
