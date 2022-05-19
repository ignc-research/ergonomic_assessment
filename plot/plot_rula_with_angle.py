from email.mime import base
import os
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib import gridspec

def dif_percent(score, base_score, label):
    """calculate procentual difference between score and base score"""
    counter = 0
    for i in range(len(score)):
        if(score[i] != base_score[i]):
            counter += 1

    percent = (counter / len(score)) * 100
    print ('{} : {}%'.format(label, percent))

def plotting_tool(time, score, base_score, conf, label, file):
    """plot score, base score and confidence"""
    sns.set_style("whitegrid")

    palette = sns.color_palette("muted")
    blue = palette[0]
    red=palette[3]

    dif_percent(score, base_score, label)

    fig = plt.figure(figsize=(15, 7), dpi=100)
    gs = gridspec.GridSpec(2, 1, hspace=0.05)
    ax0 = fig.add_subplot(gs[0])
    
    # ax0.plot(time, score, '-o', lw=0.8, ms=2, color=blue, label="Mit Gewichtung nach Konfidenz")
    # ax0.plot(time, base_score, '--', lw=0.8, color=red, label="Ohne Konfidenz")
    # ax0.fill_between(time, score, alpha=0.2)
    ax0.plot(time, score, '-o', drawstyle='steps-post', lw=0.8, ms=2, color=blue, label="Mit Gewichtung nach Konfidenz")    #plot with steps
    ax0.plot(time, base_score, '--', drawstyle='steps-post', lw=0.8, color=red, label="Ohne Konfidenz")    #plot with steps
    ax0.fill_between(time, score, step='post', alpha=0.2)    #plot with steps

    ax0.set_ylim(ymin=1)
    ax0.legend(loc="upper left", fontsize=8)

    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    
    # ax1.plot(time, conf, lw=0.8, color=palette[7])
    ax1.plot(time, conf, drawstyle='steps-post', lw=0.8, color=palette[7])    #plot with steps
    
    ax1.set_ylim(ymax=1)

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
    
    ax0.margins(x=0)

    fig.savefig(file + '.png')

def plotting_tool_angle(time, score, base_score, conf, angle, label, file):
    """plot score, base score, confidence and angle"""
    sns.set_style("whitegrid")

    palette = sns.color_palette("muted")
    blue = palette[0]
    red=palette[3]

    #round Scores to whole number and limit to a minimum of 1
    for i in range(len(score)):
        if (score[i] < 1):
            score[i] = 1
        else:
            score[i] = int(round(score[i]))        

    dif_percent(score, base_score, label)

    fig = plt.figure(figsize=(15, 7), dpi=100)
    gs = gridspec.GridSpec(3, 1, hspace=0.05)
    ax0 = fig.add_subplot(gs[0])
    
    # ax0.plot(time, score, '-o', lw=0.8, ms=2, color=blue, label="Mit Gewichtung nach Konfidenz")
    # ax0.plot(time, base_score, '--', lw=0.8, color=red, label="Ohne Konfidenz")
    # ax0.fill_between(time, score, alpha=0.2)
    ax0.plot(time, score, '-o', drawstyle='steps-post', lw=0.8, ms=2, color=blue, label="Mit Gewichtung nach Konfidenz")    #plot with steps
    ax0.plot(time, base_score, '--', drawstyle='steps-post', lw=0.8, color=red, label="Ohne Konfidenz")    #plot with steps
    ax0.fill_between(time, score, step='post', alpha=0.2)    #plot with steps
   
    ax0.margins(x=0)
    ax0.set_ylim(ymin=0)
    ax0.legend(loc="upper left", fontsize=8)

    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    
    # ax1.plot(time, conf, lw=0.8, color=palette[7])
    ax1.plot(time, conf, drawstyle='steps-post', lw=0.8, color=palette[7])    #plot with steps
    
    ax1.set_ylim(ymax=1.05)
    
    ax2 = fig.add_subplot(gs[2], sharex=ax0)
    
    # ax2.plot(time, angle, lw=0.8, color=palette[7])
    ax2.plot(time, angle, drawstyle='steps-post', lw=0.8, color=palette[7])    #plot with steps
    
    plt.setp(ax0.get_xticklabels(), visible=False)
    plt.setp(ax1.get_xticklabels(), visible=False)

    ax0.tick_params(axis='both', which='major', labelsize=8)
    ax1.tick_params(axis='both', which='major', labelsize=8)
    ax2.tick_params(axis='both', which='major', labelsize=8)

    plt.xlabel("Zeit in Sekunden", fontsize=8)
    ax0.set_ylabel(label, fontsize=8)
    ax1.set_ylabel('Konfidenz', fontsize=8)
    ax2.set_ylabel('Winkel in Grad', fontsize=8)
    ax0.xaxis.grid(which='minor') # vertical lines
    ax1.xaxis.grid(which='minor') # vertical lines
    ax2.xaxis.grid(which='minor') # vertical lines
    ax0.minorticks_on()
    ax0.grid('on')
    ax1.grid('on')
    ax2.grid('on')

    fig.savefig(file + '.png')

def plot_rula():

    rula = np.loadtxt("../data/data_for_plot.csv", delimiter=' ', skiprows=1)      #load data from path and skip first row that contains entry names
    time = rula[:,0]
    time = (time - time[0])     #time relative to first entry
    time = [round(elem, 2) for elem in time]

    folder = "rula_plots_"+datetime.now().strftime("%Y%m%d_%H%M%S")+"/"     #unique folder names
    if not os.path.exists(folder):      #creates target folder if it doesn't exists
        os.makedirs(folder)
    
    print('Unterschiede zwischen Punktzahl mit Gewichtung und ohne Gewichtung (in Prozent):')       #first terminal line for dif_percent

    rula_score = rula[:,1]
    rula_base_score = rula[:,2]
    rula_conf = rula[:,3]
    plotting_tool(time, rula_score, rula_base_score, rula_conf, 'RULA Punktzahl', folder+'rula')

    neck_score = rula[:,4]
    neck_base_score = rula[:,5]
    neck_confidence = rula[:,6]
    neck_angle = rula[:,7]
    plotting_tool_angle(time, neck_score, neck_base_score, neck_confidence, neck_angle, 'Hals Punktzahl', folder+'hals')

    trunk_score = rula[:,8]
    trunk_base_score = rula[:,9]
    trunk_confidence = rula[:,10]
    trunk_angle = rula[:,11]
    plotting_tool_angle(time, trunk_score, trunk_base_score, trunk_confidence, trunk_angle, 'Oberköper Punktzahl', folder+'oberkorper')

    legs_score = rula[:,12]
    legs_base_score = rula[:,13]
    legs_confidence = rula[:,14]
    legs_angle = rula[:,15]
    plotting_tool_angle(time, legs_score, legs_base_score, legs_confidence, legs_angle, 'Beine Punktzahl', folder+'beine')

    upper_arm_left_score = rula[:,16]
    upper_arm_left_base_score = rula[:,17]
    upper_arm_left_confidence = rula[:,18]
    upper_arm_left_angle = rula[:,19]
    plotting_tool_angle(time, upper_arm_left_score, upper_arm_left_base_score, upper_arm_left_confidence, upper_arm_left_angle, 'Oberarm Links Punktzahl', folder+'oberarm_links')

    upper_arm_right_score = rula[:,20]
    upper_arm_right_base_score = rula[:,21]
    upper_arm_right_confidence = rula[:,22]
    upper_arm_right_angle = rula[:,23]
    plotting_tool_angle(time, upper_arm_right_score, upper_arm_right_base_score, upper_arm_right_confidence, upper_arm_right_angle, 'Oberarm Rechts Punktzahl', folder+'oberarm_rechts')

    lower_arm_left_score = rula[:,24]
    lower_arm_left_base_score = rula[:,25]
    lower_arm_left_confidence = rula[:,26]
    lower_arm_left_angle = rula[:,27]
    plotting_tool_angle(time, lower_arm_left_score, lower_arm_left_base_score, lower_arm_left_confidence, lower_arm_left_angle, 'Unterarm Links Punktzahl', folder+'unterarm_links')

    lower_arm_right_score = rula[:,28]
    lower_arm_right_base_score = rula[:,29]
    lower_arm_right_confidence = rula[:,30]
    lower_arm_right_angle = rula[:,31]
    plotting_tool_angle(time, lower_arm_right_score, lower_arm_right_base_score, lower_arm_right_confidence, lower_arm_right_angle, 'Unterarm Rechts Punktzahl', folder+'unterarm_rechts')

    wrist_left_score = rula[:,32]
    wrist_left_base_score = rula[:,33]
    wrist_left_confidence = rula[:,34]
    wrist_left_angle = rula[:,35]
    plotting_tool_angle(time, wrist_left_score, wrist_left_base_score, wrist_left_confidence, wrist_left_angle, 'Handgelenk Links Punktzahl', folder+'handgelenk_links')

    wrist_right_score = rula[:,36]
    wrist_right_base_score = rula[:,37]
    wrist_right_confidence = rula[:,38]
    wrist_right_angle = rula[:,39]
    plotting_tool_angle(time, wrist_right_score, wrist_right_base_score, wrist_right_confidence, wrist_right_angle, 'Handgelenk Rechts Punktzahl', folder+'handgelenk_rechts')

if __name__ == '__main__':
    plot_rula()