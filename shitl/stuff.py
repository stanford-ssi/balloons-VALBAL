import re
filename = "../src/Avionics.cpp"
file = open(filename, 'r')
lines = file.readlines()
for i,line in enumerate(lines):
    if re.search('diddlybop', line, re.I):
    	num_report = int(re.findall(r"\[(.)\]",lines[i+1])[0])
    	names = [re.findall(r"\= (.*)\;",k)[0] for k in lines[i+2:i+2+num_report]]
    	print(names)
