import os
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib import gridspec

def plotting_tool(time, score, base_score, conf, label, file):
    sns.set_style("whitegrid")

    palette = sns.color_palette("muted")
    blue = palette[0]
    red=palette[3]

    fig = plt.figure(figsize=(15, 7), dpi=100)
    gs = gridspec.GridSpec(2, 1, hspace=0.05)
    ax0 = fig.add_subplot(gs[0])
    ax0.plot(time, score, lw=0.8, color=blue, label="Mit Gewichtung nach Konfidenz")
    ax0.plot(time, base_score, '--', lw=0.8, color=red, label="Ohne Konfidenz")
    ax0.set_ylim(ymin=0)


    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    ax1.plot(time, conf, lw=0.8, color=palette[7])
    ax1.set_ylim(ymax=1.05)
    ax0.legend(loc="upper left", fontsize=8)

    plt.setp(ax0.get_xticklabels(), visible=False)

    ax0.tick_params(axis='both', which='major', labelsize=8)
    ax1.tick_params(axis='both', which='major', labelsize=8)

    plt.xlabel("Zeit in Sekunden", fontsize=8)
    ax0.set_ylabel(label, fontsize=8)
    ax1.set_ylabel('Konfidenz', fontsize=8)
    ax0.xaxis.grid(which='minor') # vertical lines
    ax1.xaxis.grid(which='minor') # vertical lines
    ax0.minorticks_on()
    ax0.grid('on')
    ax1.grid('on')
    l = ax0.fill_between(time, score, alpha=0.2)

    ax0.margins(x=0)

    fig.savefig(file + '.png')

def plot_rula():

    rula = np.loadtxt("../data/rula_data_20220506_014429.csv", delimiter=' ', skiprows=1)
    time = rula[:,0]
    time = (time - time[0])
    time = [round(elem, 2) for elem in time]

    folder = "rula_plots_"+datetime.now().strftime("%Y%m%d_%H%M%S")+"/"
    if not os.path.exists(folder):
        os.makedirs(folder)
    
    rula_score = rula[:,1]
    rula_base_score = rula[:,2]
    rula_conf = rula[:,3]
    plotting_tool(time, rula_score, rula_base_score, rula_conf, 'RULA Punktzahl', folder+'rula')

    neck_score = rula[:,4]
    neck_base_score = rula[:,5]
    neck_confidence = rula[:,6]
    plotting_tool(time, neck_score, neck_base_score, neck_confidence, 'Hals Punktzahl', folder+'hals')

    trunk_score = rula[:,8]
    trunk_base_score = rula[:,9]
    trunk_confidence = rula[:,10]
    plotting_tool(time, trunk_score, trunk_base_score, trunk_confidence, 'Oberköper Punktzahl', folder+'oberkorper')

    legs_score = rula[:,12]
    legs_base_score = rula[:,13]
    legs_confidence = rula[:,14]
    plotting_tool(time, legs_score, legs_base_score, legs_confidence, 'Beine Punktzahl', folder+'beine')

    upper_arm_left_score = rula[:,16]
    upper_arm_left_base_score = rula[:,17]
    upper_arm_left_confidence = rula[:,18]
    plotting_tool(time, upper_arm_left_score, upper_arm_left_base_score, upper_arm_left_confidence, 'Oberarm Links Punktzahl', folder+'oberarm_links')

    upper_arm_right_score = rula[:,20]
    upper_arm_right_base_score = rula[:,21]
    upper_arm_right_confidence = rula[:,22]
    plotting_tool(time, upper_arm_right_score, upper_arm_right_base_score, upper_arm_right_confidence, 'Oberarm Rechts Punktzahl', folder+'oberarm_rechts')

    lower_arm_left_score = rula[:,24]
    lower_arm_left_base_score = rula[:,25]
    lower_arm_left_confidence = rula[:,26]
    plotting_tool(time, lower_arm_left_score, lower_arm_left_base_score, lower_arm_left_confidence, 'Unterarm Links Punktzahl', folder+'unterarm_links')

    lower_arm_right_score = rula[:,28]
    lower_arm_right_base_score = rula[:,29]
    lower_arm_right_confidence = rula[:,30]
    plotting_tool(time, lower_arm_right_score, lower_arm_right_base_score, lower_arm_right_confidence, 'Unterarm Rechts Punktzahl', folder+'unterarm_rechts')

    wrist_left_score = rula[:,32]
    wrist_left_base_score = rula[:,33]
    wrist_left_confidence = rula[:,34]
    plotting_tool(time, wrist_left_score, wrist_left_base_score, wrist_left_confidence, 'Handgelenk Links Punktzahl', folder+'handgelenk_links')

    wrist_right_score = rula[:,36]
    wrist_right_base_score = rula[:,37]
    wrist_right_confidence = rula[:,38]
    plotting_tool(time, wrist_right_score, wrist_right_base_score, wrist_right_confidence, 'Handgelenk Rechts Punktzahl', folder+'handgelenk_rechts')

if __name__ == '__main__':
    plot_rula()