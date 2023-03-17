import pandas as pd
import seaborn as sns
import seaborn_image as isns
import matplotlib.pyplot as plt
import numpy as np
import re
from matplotlib import rc

fontsize = 28

def delaystr(i):
    if i%2 == 0:
        return f"delay {int(i/2)}s"
    return f"delay {i/2}s"


def plot_vel(filenames):
    """plots the velocity over time for different delays

    Args:
        filenames (array): filenames of the different delay files (ascending in delay)
    """
    plt.rcParams.update({'font.size': 16})

    vels = pd.DataFrame()
    active = pd.DataFrame()
    '''remove timesteps where the shielding has not started yet'''
    for i, f in enumerate(filenames):
        df = pd.read_csv(f)
        df.drop(df.columns[0], axis=1, inplace=True)
        df.dropna(inplace=True)
        df = df.iloc[1:]
        vels.insert(vels.shape[1], delaystr(i), df["ego_vel"])
        active.insert(active.shape[1], delaystr(i), df["shield_takeover"])

    vels.insert(0, 'time', np.arange(0, vels.shape[0]/2, 0.5))
    vels = vels.melt('time', var_name='delays', value_name='velocity')

    active.replace({"[ True]": 1, "[False]": 0}, inplace=True)
    active.insert(0, 'time', np.arange(0, active.shape[0]/2, 0.5))
    actives = []
    '''insert active shield periodes and artificial points to plot nice rising/falling edges'''
    for i in range(active.shape[1]-1):
        a_list = list(active[delaystr(i)])
        actives.append(active[["time", delaystr(i)]].copy())
        for j in range(len(a_list)):
            if j + 1 < len(a_list) and a_list[j] == 0 and a_list[j+1] == 1:
                actives[i] = actives[i].append({"time": j/2+0.5-0.001, delaystr(i): 0}, ignore_index=True)
            if j + 1 < len(a_list) and a_list[j] == 1 and a_list[j+1] == 0:
                actives[i] = actives[i].append({"time": j/2+0.5-0.001, delaystr(i): 1}, ignore_index=True)
        actives[i].sort_values(by=["time"], inplace=True)

    fig, axs = plt.subplots(active.shape[1], 1, figsize=(7,5), gridspec_kw={'height_ratios': [10]+[1 for _ in range(len(filenames))]}, sharex=True)
    axs[0].set_xlim(0, 12)
    axs[0].set_ylim(0, 7)
    for spine in ['right', 'top']:
        axs[0].spines[spine].set_visible(False)
    # axs[0].set(ylabel=r"velocity in m/s")
    axs[0].set_ylabel("velocity (m/s)", fontsize = fontsize)
    axs[active.shape[1]-1].set_xlabel("time (s)", fontsize = fontsize)
    axs[active.shape[1]-1].xaxis.set_ticks(np.arange(0, 12.5, 1))
    sns.lineplot(x="time", y="velocity", hue='delays', style="delays", data=vels, ax=axs[0], linewidth=2)
    axs[0].legend(loc='lower left', fontsize = int(fontsize*4/5))
    for i, a in enumerate(actives):
        sns.lineplot(x="time", y=delaystr(i), color=sns.color_palette()[i], data=a, ax=axs[i+1])
        axs[i+1].set_ylim(0,1)
        axs[i+1].fill_between(a["time"], a[delaystr(i)], color=sns.color_palette()[i], hatch="//", alpha=0.3)
        axs[i+1].set(ylabel="")
        axs[i+1].legend([delaystr(i)], loc='center left', fontsize = int(fontsize*3/5))
    fig.tight_layout()
    fig.subplots_adjust(hspace=0.3)
    plt.savefig('experiments_data/plot_crossing.pdf')
    plt.show()

def plot_vel_dist_shield_active(filenames):
    """plots the distance velocity pairs when the shield took over for different delays
       plot the regression line and CI through these points

    Args:
        filenames (array): filenames of the different delay files (ascending in delay)
    """
    active = pd.DataFrame(columns=["shield_takeover", "edi", "ev", "delay"])
    '''remove timesteps where the shielding has not started yet
       move shield_takeover delay steps back, s.t. shield_takeover matches up with the observations'''

    for i, f in enumerate(filenames):
        df = pd.read_csv(f)
        df.drop(df.columns[0], axis=1, inplace=True)
        df.dropna(inplace=True)
        df = df.iloc[1:]
        df["shield_takeover"] = df["shield_takeover"].shift(-i)
        df.dropna(inplace=True)
        for st, ed, ev, od in zip(df["shield_takeover"], df["ego_dist"], df["ego_vel"], df["other_dist"]):
            st = re.sub("  ", " ", st)
            st = re.sub("\[ ", "[", st)
            st = re.sub("\[", "", st)
            st = re.sub("\]", "", st)
            ed = re.sub("\[", "", ed)
            ed = re.sub("\]", "", ed)
            ed = re.split(r'\s+',ed)
            ed = [i for i in ed if i != ""]
            od = re.sub("\[", "", od)
            od = re.sub("\]", "", od)
            od = re.split(r'\s+',od)
            od = [i for i in od if i != ""]
            for st_i, ed_i, od_i in zip(st.split(" "), ed, od):
                # if float(od_i) > float(ed_i):
                #     continue
                active = active.append({"shield_takeover": st_i == "True", "edi": float(ed_i), "ev": ev, "delay": delaystr(i)}, ignore_index=True)
    active = active[active["shield_takeover"] == True]

    g = sns.lmplot(data=active, x="ev", y="edi", hue="delay", order=2, markers=['o','X','^'],
                   palette="deep", legend=False, height=4, aspect=13/5, truncate=True, scatter_kws={'s': 200})
    # sns.kdeplot(data=active, x="ego_vel", y="ego_dist", hue="delay", levels=2, palette="deep")
    g.set(xlabel="velocity (m/s)")
    g.set(ylabel="distance (m)")

    for ax in g.axes.flat:
        # Make x and y-axis labels slightly larger
        ax.set_xlabel(ax.get_xlabel(), fontsize=fontsize) #was 24
        ax.set_ylabel(ax.get_ylabel(), fontsize=fontsize) #was 24
    g.ax.legend(loc='best', fontsize=int(fontsize*4/5))
    g.ax.lines[1].set_linestyle('dashed')
    g.ax.lines[2].set_linestyle('dotted')
    g.set(xlim=(-0.2, active["ev"].max()+0.5))
    #g.set(ylim=(0, active["edi"].max()+10))
    g.set(ylim=(0,35))
    #g.set(yticks=(np.arange(0, active["edi"].max()+10, 10)))
    g.set(yticks=(np.arange(0, 35, 10)))
    plt.tight_layout()
    plt.savefig('experiments_data/plot_pedestrian.pdf')
    plt.show()

def main():
    sns.set_style("ticks")
    sns.set_context("paper")

    rc('font', **{'family': 'serif', 'serif': ['Times New Roman'], 'weight':'bold'})
    rc('text', usetex=True)

    # filenames = ["./experiment/crossing_d0.csv", "./experiment/crossing_d1.csv", "./experiment/crossing_d2.csv", "./experiment/crossing_d3.csv"]
    filenames = ["./experiments_data/crossing_d0.csv", "./experiments_data/crossing_d1.csv", "./experiments_data/crossing_d2.csv", "./experiments_data/crossing_d3.csv"]
    # filenames = ["./experiments_data/crossing_d0.csv", "./experiments_data/crossing_d1.csv", "./experiments_data/crossing_d2.csv"]
    plot_vel(filenames)
    sns.set(font_scale=2)
    sns.set_style("white")
    rcParams = {}
    rcParams['font.family'] = 'serif'
    rcParams['font.serif'] = ['Times New Roman']
    rcParams['font.size'] = 28
    rcParams["font.weight"] = "bold"
    rcParams["axes.labelweight"] = "bold"
    isns.set_context(rc=rcParams)

    # filenames = ["./experiment/pedestrian_d0.csv", "./experiment/pedestrian_d1.csv", "./experiment/pedestrian_d2.csv"]
    filenames = ["./experiments_data/pedestrian_d0.csv", "./experiments_data/pedestrian_d1.csv", "./experiments_data/pedestrian_d2.csv"]
    plot_vel_dist_shield_active(filenames)

if __name__ == '__main__':
    main()
