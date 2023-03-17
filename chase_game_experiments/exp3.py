import pandas as pd
from matplotlib import rc
from matplotlib import pyplot as plt

plt.rcParams.update({'font.size': 20})
rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
rc('text', usetex=True)

with open('exp3.txt', 'r') as file:
    data = file.read()

n_rounds = 0
n_steps = 0
for line in data.split('\n'):
    if 'rounds' in line and 'steps' in line:
        n_rounds = int(line.split('strategy,')[-1].split('rounds')[0])
        n_steps = int(line.split('with')[-1].split('steps')[0])

cols = ['delta', 'pre_shield','safest', 'most_controllable']
df_score = pd.DataFrame(columns=cols)
df_inter = pd.DataFrame(columns=cols)
delta = 0
current_strategy = 'pre_shield'

for line in data.split('\n')[1:]:
    if 'safest strategy' in line:
        current_strategy = 'safest'
        continue
    if 'most_controllable strategy' in line:
        current_strategy = 'most_controllable'
        continue
    if 'trivial strategy' in line:
        current_strategy = 'pre_shield'
        continue
    if len(line) < 5 or not ('score' in line):
        continue

    delta = int(line.split('delta= ')[-1].split(',')[0])
    df_score.loc[delta,'delta'] = delta
    df_inter.loc[delta,'delta'] = delta

    newscore = int(line.split('score:')[-1].split(',')[0])
    newinter = int(line.split('interference:')[-1])
    df_score.loc[delta, current_strategy] = newscore/n_rounds
    df_inter.loc[delta, current_strategy] = newinter/n_rounds




figfactor = 0.5
fig, ax1 = plt.subplots(figsize=(15*figfactor,10*figfactor))
#ax1.plot(df_score.delta.astype('int'), df_score.pre_shield, marker = 'o', color = 'red')
ax1.plot(df_score.delta.astype('int'), df_score.safest, marker = '^', color = 'green')
ax1.plot(df_score.delta.astype('int'), df_score.most_controllable, marker = 's', color = 'blue')
#ax1.plot(df_score.delta.astype('int'), df_inter.pre_shield, marker = 'o', color = 'darkred')
ax1.plot(df_score.delta.astype('int'), df_inter.safest, marker = '^', color = 'darkgreen')
ax1.plot(df_score.delta.astype('int'), df_inter.most_controllable, marker = 's', color = 'darkblue')
ax1.set_xlabel('delay')
ax1.set_ylabel('     score $\qquad\quad$      shield interventions')
ax1.set_yscale('log', base=10)
ax1.set_xticks(df_score.delta.astype('int'))
#ax1.legend(['Pre-Shield', 'Robust-Safety', 'Delay-Resilient'])
ax1.legend(['Robust-Safety', 'Delay-Resilient'])
plt.savefig('exp3.pdf', bbox_inches='tight')
plt.show()
