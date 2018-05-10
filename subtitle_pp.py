# -*- coding: utf-8 -*-
"""
Created on Thu May  3 11:00:38 2018

@author: atpandey
"""
#%%
import os
#list of srt files

lista=os.listdir()
lista.sort()
def process_transcript(fname):
    with open(fname) as ff:
        for line in ff:
            line=line.rstrip()
            if len(line) ==0 or len(line) <= 2:
                pass
            elif (line[2]==':'):
                pass
            else:
                print(line,end=' ')
    ff.close()
        
#%%
#fname='1 - 01 L Lesson Outline - lang_en_vs1.srt'
fname=lista[0]
print(fname)
print("*******************")
#print("operating on",fname)
process_transcript(fname)


#%%
fname=lista[3]
print(fname)
print("*******************")
process_transcript(fname)

#%%
fname=lista[4]
print(fname)
print("*******************")
process_transcript(fname)

#%%
fname=lista[5]
print(fname)
print("*******************")
process_transcript(fname)

#%%
fname=lista[6]
print(fname)
print("*******************")
process_transcript(fname)
#%%
fname=lista[7]
print(fname)
print("*******************")
process_transcript(fname)

#%%
fname=lista[8]
print(fname)
print("*******************")
process_transcript(fname)

#%%
fname=lista[9]
print(fname)
print("*******************")
process_transcript(fname)

#%%
fname=lista[10]
print(fname)
print("*******************")
process_transcript(fname)
#%% 
#10th lesson is 1st in lista
#fname='2 - 03 L The Behavior Problem - lang_en_vs1.srt'
fname=lista[1]
print(fname)
print("*******************")
#print("operating on",fname)
process_transcript(fname)
#%%
#11th lesson is 2nd in lista
fname=lista[2]
print(fname)
print("*******************")
process_transcript(fname)