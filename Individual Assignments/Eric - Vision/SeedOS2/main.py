import PySimpleGUI as sg
import os

import generateSettings
import generateCalibration
import generateCenter
import generatePath

#Deisgned to be run with a 1080p display
#Create Window using PySimpleGui
sg.theme('DarkAmber')   # Set Theme
#Create the layout for the window
directions = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum."
left_col  = [[sg.Graph((700,700) , (0,0) , (700,700), background_color='white', key='-GRAPH-')]]
right_col = [[sg.Text('Directions:', font='Any 15')],
             [sg.Text(directions, size=(60, None))],
             [sg.Text(' ', font='Any 20')],
             [sg.Text('Camera Settings', font='Any 15')],
             [sg.Button('Create', key='-setting_create-'), sg.Button('Load', key='-setting_load-'), sg.InputText('Camera_Settings.yaml', key='-setting_browse-'), sg.FileBrowse()],
             [sg.Text('Camera Calibration', font='Any 15')],
             [sg.Button('Create', key='-calibrate_create-'), sg.Button('Load', key='-calibrate_load-'), sg.InputText('Camera_Calibration.yaml', key='-calibration_browse-'), sg.FileBrowse()],
             [sg.Text('Optical Center', font='Any 15')],
             [sg.Button('Create', key='-center_create-'), sg.Button('Load', key='-center_load-'), sg.InputText('Optical_Center.yaml', key='-center_browse-'), sg.FileBrowse()],
             [sg.Text('Path Planning', font='Any 15')],
             [sg.Button('Create', key='-path_create-'), sg.Button('Load', key='-path_load-'), sg.InputText('ArUco_Path.yaml', key='-path_browse-'), sg.FileBrowse()],
             [sg.Text(' ', font='Any 20')],
             [sg.Button('Load All'), sg.Button('Run Robot'), sg.Cancel()],]
layout = [[sg.Text('ArUco Based Path Following', font='Any 20')],
          [sg.Text('Devleoped by: Name 1, Name 2, Name 3, Name 4', font='Any 10')],
          [sg.Column(left_col),sg.Column(right_col)],
         ]
# Create the Window with the above Layout
window = sg.Window('ArUco Based Path Following', layout)


def run():
    pass
    
def initialize():
    pass

def cleanup():
    window.close()

def main():
    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED: # if user closes window or clicks cancel
            break
        elif event == '-calibrate_create-':
            try:
                generateCalibration.main();
            except:
                print("An Error has occured while trying to generate a calibration file")
        #print('You entered ', values[0])


if __name__ == '__main__':
    initialize()
    main()
    cleanup()



#Integrate PyGame with PySimpleGui
#https://pythonprogramming.altervista.org/how-to-integrate-pysimplegui-with-pygame/amp/
#Create Columns within pySimpleGui
#https://csveda.com/pysimplegui-column-and-frame/
#Textwrap within Python
#https://stackoverflow.com/questions/67353402/pysimplegui-how-to-get-string-to-print-on-new-line-instead-of-cutting-it-off



#Vars to set
#Dictionary
#Calibration Constants

