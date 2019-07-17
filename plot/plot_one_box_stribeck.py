import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib
matplotlib.use('qt5agg')
from matplotlib import pyplot as plt
import sys
from ReadCurrents import LoadCurrents

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

def butter_highpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a


def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y


if __name__ == '__main__': 
    
    if len(sys.argv) != 6: 
        print '**Usage: %s <filename> <outfolder> <plotname> <f> <outraw>' %(sys.argv[0])
        sys.exit()

    filename = sys.argv[1] #~/dynamixel/April122019/out_april122019.csv
    outfolder = sys.argv[2]
    plotname = sys.argv[3]
    [times, currents, RMS_currents] = LoadCurrents(filename)
    order = 4
    freq = float(sys.argv[4])
    Fs = 1.0 * (times.shape[0] - 1)/ (times[-1] - times[0]) 
    outname = sys.argv[5]
    print(Fs)
    # highpass filter RMS current
    highpass_RMS = butter_highpass_filter(RMS_currents, freq, Fs, order=order)
    highpass_1 = butter_highpass_filter(currents[:,0], freq, Fs, order=order)
    highpass_2 = butter_highpass_filter(currents[:,1], freq, Fs, order=order)
    highpass_3 = butter_highpass_filter(currents[:,2], freq, Fs, order=order)
    highpass_4 = butter_highpass_filter(currents[:,3], freq, Fs, order=order)

    # make plots
    data = np.zeros((RMS_currents.shape[0],2))
    data[:,0] = times;
    data[:,1] = highpass_RMS;
    np.savetxt(outname, data, delimiter=',',header='time, highpass_RMS', comments='')
    #original current data
    fig = plt.figure(figsize=(14,10))
    
    plt.subplot(1,2,1)
    plt.plot(times,currents[:,0], label = 'Current 1')
    plt.plot(times,currents[:,1], label= 'Current 2')
    plt.plot(times,currents[:,2], label= 'Current 3')
    plt.plot(times,currents[:,3], label= 'Current 4')
    plt.plot(times,RMS_currents, label= 'Current L2Norm')
    plt.xlabel('Time (s)')
    plt.ylabel('Scaled Current')
    plt.legend(loc='upper right')
    plt.title('Raw')
    #plt.yscale('log')
    #plt.suptitle('Multipole Coefficient ((n,m) = (%d,%d)), Obj %d' %(n,m, obj))
    #ymin = np.power(10.0, np.floor(np.log10(np.amin(plot_y))))
    #ymax = np.power(10.0, np.ceil(np.log10(np.amax(plot_y))))
    #plt.ylim(ymin, ymax)
    #print(np.amin(avs))
    #print(np.amax(avs))
    #plt.ylim([np.amin(avs),np.amax(avs)])
    plt.subplot(1,2,2)
    plt.plot(times,RMS_currents, label= 'Current L2Norm')
    plt.plot(times,highpass_RMS, label = 'Highpass L2Norm')
    plt.xlabel('Time (s)')
    plt.ylabel('Scaled Current')
    plt.legend(loc='upper right')
    plt.title('HighPass %.1f Hz Order %d' %(freq, order))
    plt.suptitle('Current Data on %s' %(plotname))
    plt.savefig('%s/%s_hf%d.pdf' %(outfolder, plotname,int(freq)))
    plt.show()
    plt.close(fig)
    
