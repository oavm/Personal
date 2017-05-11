

def makePlots(vars_to_plot, plot_number):
    '''
    This function creates three plots for joint position, velocity, and effort.
    Each plot displays signals for measured values and commanded values.
    The input argument is a list of strings representing the joint names to plot.
    '''

    # string arrays for EST_ROBOT_STATE and ATLAS_COMMAND
    debug_names = msg.name


  
    for var in vars:
        addSignal('debug', msg.utime, msg.darray[debug_names[var]], debug_names, keyLookup=var, plot=plot_number)
    

          
#def plot_number(number):
# 	out_str = 'p'
#	for count in range(1,number)   
#    out_str = addPlot()
#	return out_str
          
