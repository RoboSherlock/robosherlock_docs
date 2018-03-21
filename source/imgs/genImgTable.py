from os import listdir
from os.path import isfile,join,isdir

mypath = "objects"
path_to_cad = "/home/presentation/work/odu_iai/cad_models"
path_to_partial = "/home/presentation/work/odu_iai/object_data/partial_views"

onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
cadfiles  = [f for f in listdir(path_to_cad) if isdir(join(path_to_cad, f))]
print cadfiles
ttfiles   = [f for f in listdir(path_to_partial) if isdir(join(path_to_partial, f))]
print ttfiles
onlyfiles.sort()

def find_longest_word(word_list):
	longest_word = ''
	for word in word_list:
 		if len(word) > len(longest_word):
			longest_word = word
	return len(longest_word)

out_file = open('image_table','w')
for f in onlyfiles:
	out_file.write('.. |'+f[:-4]+'| image:: imgs/objects/'+f+'\n')
	out_file.write('   :width: 80pt\n')

longest= find_longest_word(onlyfiles)

out_file.write('\n\n')


out_file.write('+'+"-"*longest+'+'+"-"*(longest+1)+'+---------+---------+\n')
out_file.write('+'+"="*longest+'+'+"="*(longest+1)+'+=========+=========+\n')
for f in onlyfiles:
	flen = len(f)
	cad = 'no'
 	tt = 'no'
        if f[:-4] in cadfiles:
		cad='yes'
        if f[:-4] in ttfiles:
		tt='yes'
	out_file.write('|'+f[:-4].ljust(longest)+'||'+(f[:-4]+'|').ljust(longest)+'|'+cad.ljust(9)+'|'+tt.ljust(9) +'|\n')
        out_file.write('+'+"-"*longest+'+'+"-"*(longest+1)+'+---------+---------+\n')
out_file.close()
