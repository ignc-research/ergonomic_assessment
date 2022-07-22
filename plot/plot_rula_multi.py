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

def plotting_tool_rula(time, left_score, left_base, left_conf, mid_score, mid_base, mid_conf, right_score, right_base, right_conf, file):
    """plot rula score, base score and confidence"""
    sns.set_style("whitegrid")

    palette = sns.color_palette("muted")
    blue = palette[0]
    red=palette[3]

    marker_size = 4
    line_width = 2
    font_size_plot = 14
    font_size_label = 16
    # dif_percent(score, base_score, label)
    # averages(score, conf, label)

    fig = plt.figure(figsize=(17, 8), dpi=100)
    gs = gridspec.GridSpec(2, 3, hspace=0.1)

    ax0 = fig.add_subplot(gs[0,0])
    ax0.plot(time, left_score, '-o', drawstyle='steps-post', lw=line_width, ms=marker_size, color=blue, label="Mit Conf.")    #plot with steps
    ax0.plot(time, left_base, '--', drawstyle='steps-post', lw=line_width, color=red, label="Ohne Conf.")    #plot with steps
    ax0.fill_between(time, left_score, step='post', alpha=0.2)    #plot with steps
    ax0.set_ylim(ymin=1, ymax=8)
    ax0.legend(loc="upper left", fontsize=font_size_plot)

    ax2 = fig.add_subplot(gs[0,1])
    ax2.plot(time, mid_score, '-o', drawstyle='steps-post', lw=line_width, ms=marker_size, color=blue, label="Mit Conf.")    #plot with steps
    ax2.plot(time, mid_base, '--', drawstyle='steps-post', lw=line_width, color=red, label="Ohne Conf.")    #plot with steps
    ax2.fill_between(time, mid_score, step='post', alpha=0.2)    #plot with steps
    ax2.set_ylim(ymin=1, ymax=8)

    ax4 = fig.add_subplot(gs[0,2])
    ax4.plot(time, right_score, '-o', drawstyle='steps-post', lw=line_width, ms=marker_size, color=blue, label="Mit Conf.")    #plot with steps
    ax4.plot(time, right_base, '--', drawstyle='steps-post', lw=line_width, color=red, label="Ohne Conf.")    #plot with steps
    ax4.fill_between(time, right_score, step='post', alpha=0.2)    #plot with steps
    ax4.set_ylim(ymin=1, ymax=8)

    ax1 = fig.add_subplot(gs[1,0], sharex=ax0)
    ax1.plot(time, left_conf, drawstyle='steps-post', lw=line_width, color=palette[7])    #plot with steps
    ax1.set_ylim(ymax=1, ymin=0.2)

    ax3 = fig.add_subplot(gs[1,1], sharex=ax2)
    ax3.plot(time, mid_conf, drawstyle='steps-post', lw=line_width, color=palette[7])    #plot with steps
    ax3.set_ylim(ymax=1, ymin=0.2)

    ax5 = fig.add_subplot(gs[1,2], sharex=ax3)
    ax5.plot(time, right_conf, drawstyle='steps-post', lw=line_width, color=palette[7])    #plot with steps
    ax5.set_ylim(ymax=1, ymin=0.2)
    
    plt.setp(ax0.get_xticklabels(), visible=False)
    plt.setp(ax2.get_xticklabels(), visible=False)
    plt.setp(ax4.get_xticklabels(), visible=False)

    ax0.tick_params(axis='both', which='major', labelsize=font_size_plot)
    ax1.tick_params(axis='both', which='major', labelsize=font_size_plot)
    ax2.tick_params(axis='both', which='major', labelsize=font_size_plot)
    ax3.tick_params(axis='both', which='major', labelsize=font_size_plot)
    ax4.tick_params(axis='both', which='major', labelsize=font_size_plot)
    ax5.tick_params(axis='both', which='major', labelsize=font_size_plot)

    # plt.xlabel("Zeit in Sekunden", fontsize=font_size_label)
    ax0.set_ylabel('Gesamtwert', fontsize=font_size_label)
    ax1.set_ylabel('Confidence Level', fontsize=font_size_label)
    ax3.set_xlabel('Zeit in Sekunden', fontsize=font_size_label)
    ax0.set_title('left title', fontsize=font_size_label)
    ax2.set_title('middle title', fontsize=font_size_label)
    ax4.set_title('right title', fontsize=font_size_label)
    ax0.xaxis.grid(which='minor') # vertical lines
    ax1.xaxis.grid(which='minor') # vertical lines
    ax2.xaxis.grid(which='minor') # vertical lines
    ax3.xaxis.grid(which='minor') # vertical lines
    ax4.xaxis.grid(which='minor') # vertical lines
    ax5.xaxis.grid(which='minor') # vertical lines
    ax0.minorticks_on()
    ax2.minorticks_on()
    ax4.minorticks_on()
    ax0.grid('on')
    ax1.grid('on')
    ax2.grid('on')
    ax3.grid('on')
    ax4.grid('on')
    ax5.grid('on')

    ax0.margins(x=0)
    ax2.margins(x=0)
    ax4.margins(x=0)


    fig.savefig(file + '.png', bbox_inches='tight')

def plot_rula():

    f_left = np.loadtxt("../data/1_data_for_plot.csv", delimiter=' ', skiprows=1)      #load data from path and skip first row that contains entry names
    f_mid = np.loadtxt("../data/2_data_for_plot.csv", delimiter=' ', skiprows=1)
    f_right = np.loadtxt("../data/2_data_for_plot.csv", delimiter=' ', skiprows=1)
    
    
    time = f_left[:,0]
    start = time[0]     #first time value
    time = (time - start)     #time relative to first entry
    time = [round(elem, 2) for elem in time]

    folder = "rula_plots_"+datetime.now().strftime("%Y%m%d_%H%M%S")+"/"     #unique folder names
    if not os.path.exists(folder):      #creates target folder if it doesn't exists
        os.makedirs(folder)
    
    left_score = f_left[:,1]
    left_base = f_left[:,2]
    left_conf = f_left[:,3]
    mid_score = f_mid[:,1]
    mid_base = f_mid[:,2]
    mid_conf = f_mid[:,3]
    right_score = f_right[:,1]
    right_base = f_right[:,2]
    right_conf = f_right[:,3]
    plotting_tool_rula(time, left_score, left_base, left_conf, mid_score, mid_base, mid_conf, right_score, right_base, right_conf, folder+'multiplot')

if __name__ == '__main__':
    plot_rula()
