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

def averages(score, conf, label):
    avg_score = sum(score) / len(score)
    avg_conf = sum(conf) / len(conf)
    print ('{} Average : {} {}'.format(label, avg_score, avg_conf))

def plotting_tool_rula(time, score, base_score, conf, label, file):
    """plot rula score, base score and confidence"""
    sns.set_style("whitegrid")

    palette = sns.color_palette("muted")
    blue = palette[0]
    red=palette[3]

    dif_percent(score, base_score, label)
    # averages(score, conf, label)

    fig = plt.figure(figsize=(17, 8), dpi=100)
    gs = gridspec.GridSpec(2, 1, hspace=0.1)
    ax0 = fig.add_subplot(gs[0])
    
    # ax0.plot(time, score, lw=0.8, color=blue, label="Mit Gewichtung nach Konfidenz")
    # ax0.plot(time, base_score, '--', lw=0.8, color=red, label="Ohne Konfidenz")
    # ax0.fill_between(time, score, alpha=0.2)
    ax0.plot(time, score, '-o', drawstyle='steps-post', lw=1, ms=2, color=blue, label="Mit Unsicherheit")    #plot with steps
    ax0.plot(time, base_score, '--', drawstyle='steps-post', lw=1, color=red, label="Ohne Unsicherheit")    #plot with steps
    ax0.fill_between(time, score, step='post', alpha=0.2)    #plot with steps
    
    ax0.set_ylim(ymin=1, ymax=8)
    ax0.legend(loc="upper left", fontsize=14)

    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    
    # ax1.plot(time, conf, lw=0.8, color=palette[7])
    ax1.plot(time, conf, drawstyle='steps-post', lw=1, color=palette[7])    #plot with steps
    
    ax1.set_ylim(ymax=1, ymin=0.2)
    
    plt.setp(ax0.get_xticklabels(), visible=False)

    ax0.tick_params(axis='both', which='major', labelsize=14)
    ax1.tick_params(axis='both', which='major', labelsize=14)

    plt.xlabel("Zeit in Sekunden", fontsize=16)
    ax0.set_ylabel(label, fontsize=16)
    ax1.set_ylabel('Confidence Level', fontsize=16)
    ax0.xaxis.grid(which='minor') # vertical lines
    ax1.xaxis.grid(which='minor') # vertical lines
    ax0.minorticks_on()
    ax0.grid('on')
    ax1.grid('on')

    ax0.margins(x=0)

    fig.savefig(file + '.png', bbox_inches='tight')

def plotting_tool(time, score, base_score, conf, label, file):
    """plot individual score, base score and confidence"""
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

    fig = plt.figure(figsize=(17, 8), dpi=100)
    gs = gridspec.GridSpec(2, 1, hspace=0.1)
    ax0 = fig.add_subplot(gs[0])
    
    # ax0.plot(time, score, lw=0.8, color=blue, label="Mit Gewichtung nach Konfidenz")
    # ax0.plot(time, base_score, '--', lw=0.8, color=red, label="Ohne Konfidenz")
    # ax0.fill_between(time, score, alpha=0.2)
    ax0.plot(time, score, '-o', drawstyle='steps-post', lw=1, ms=2, color=blue, label="Mit Unsicherheit")    #plot with steps
    ax0.plot(time, base_score, '--', drawstyle='steps-post', lw=1, color=red, label="Ohne Unsicherheit")    #plot with steps
    ax0.fill_between(time, score, step='post', alpha=0.2)    #plot with steps
    
    ax0.set_ylim(ymin=0)
    ax0.legend(loc="upper left", fontsize=14)

    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    
    # ax1.plot(time, conf, lw=0.8, color=palette[7])
    ax1.plot(time, conf, drawstyle='steps-post', lw=1, color=palette[7])    #plot with steps
    
    ax1.set_ylim(ymax=1.05, ymin=0.2)
    
    plt.setp(ax0.get_xticklabels(), visible=False)

    ax0.tick_params(axis='both', which='major', labelsize=14)
    ax1.tick_params(axis='both', which='major', labelsize=14)

    plt.xlabel("Zeit in Sekunden", fontsize=16)
    ax0.set_ylabel(label, fontsize=16)
    ax1.set_ylabel('Confidence Level', fontsize=16)
    ax0.xaxis.grid(which='minor') # vertical lines
    ax1.xaxis.grid(which='minor') # vertical lines
    ax0.minorticks_on()
    ax0.grid('on')
    ax1.grid('on')

    ax0.margins(x=0)

    fig.savefig(file + '.png', bbox_inches='tight')

def plot_rula():

    rula = np.loadtxt("../data/avg_600.csv", delimiter=' ', skiprows=1)      #load data from path and skip first row that contains entry names
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
    plotting_tool_rula(time, rula_score, rula_base_score, rula_conf, 'RULA-Gesamtwert', folder+'rula')

    neck_score = rula[:,4]
    neck_base_score = rula[:,5]
    neck_confidence = rula[:,6]
    plotting_tool(time, neck_score, neck_base_score, neck_confidence, 'Halswert', folder+'hals')

    trunk_score = rula[:,8]
    trunk_base_score = rula[:,9]
    trunk_confidence = rula[:,10]
    plotting_tool(time, trunk_score, trunk_base_score, trunk_confidence, 'Oberköperwert', folder+'oberkorper')

    legs_score = rula[:,12]
    legs_base_score = rula[:,13]
    legs_confidence = rula[:,14]
    plotting_tool(time, legs_score, legs_base_score, legs_confidence, 'Beinwert', folder+'beine')

    upper_arm_left_score = rula[:,16]
    upper_arm_left_base_score = rula[:,17]
    upper_arm_left_confidence = rula[:,18]
    plotting_tool(time, upper_arm_left_score, upper_arm_left_base_score, upper_arm_left_confidence, 'Oberarmwert Links', folder+'oberarm_links')

    upper_arm_right_score = rula[:,20]
    upper_arm_right_base_score = rula[:,21]
    upper_arm_right_confidence = rula[:,22]
    plotting_tool(time, upper_arm_right_score, upper_arm_right_base_score, upper_arm_right_confidence, 'Oberarmwert Rechts', folder+'oberarm_rechts')

    lower_arm_left_score = rula[:,24]
    lower_arm_left_base_score = rula[:,25]
    lower_arm_left_confidence = rula[:,26]
    plotting_tool(time, lower_arm_left_score, lower_arm_left_base_score, lower_arm_left_confidence, 'Unterarmwert Links', folder+'unterarm_links')

    lower_arm_right_score = rula[:,28]
    lower_arm_right_base_score = rula[:,29]
    lower_arm_right_confidence = rula[:,30]
    plotting_tool(time, lower_arm_right_score, lower_arm_right_base_score, lower_arm_right_confidence, 'Unterarmwert Rechts', folder+'unterarm_rechts')

    wrist_left_score = rula[:,32]
    wrist_left_base_score = rula[:,33]
    wrist_left_confidence = rula[:,34]
    plotting_tool(time, wrist_left_score, wrist_left_base_score, wrist_left_confidence, 'Handgelenkwert Links', folder+'handgelenk_links')

    wrist_right_score = rula[:,36]
    wrist_right_base_score = rula[:,37]
    wrist_right_confidence = rula[:,38]
    plotting_tool(time, wrist_right_score, wrist_right_base_score, wrist_right_confidence, 'Handgelenkwert Rechts', folder+'handgelenk_rechts')

if __name__ == '__main__':
    plot_rula()
