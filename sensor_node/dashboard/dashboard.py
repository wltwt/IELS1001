import tkinter as tk

window = tk.Tk()

# setting the title and  
window.title('Plotting in Tkinter') 
  
# setting the dimensions of  
# the main window 
window.geometry("500x500")

  
# button that would displays the plot 
plot_button = tk.Button(master = window, 
                     height = 2, 
                     width = 10, 
                    text = "Plot") 
# place the button 
# into the window 
plot_button.pack() 


window.mainloop()
