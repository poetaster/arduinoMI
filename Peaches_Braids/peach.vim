let SessionLoad = 1
if &cp | set nocp | endif
let s:so_save = &g:so | let s:siso_save = &g:siso | setg so=0 siso=0 | setl so=-1 siso=-1
let v:this_session=expand("<sfile>:p")
silent only
silent tabonly
cd ~
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
let s:shortmess_save = &shortmess
if &shortmess =~ 'A'
  set shortmess=aoOA
else
  set shortmess=aoO
endif
badd +0 src/arduinoMI/Peaches_Braids/Peaches_Braids.ino
badd +0 src/arduinoMI/Peaches_Braids/braids.h
argglobal
%argdel
set stal=2
tabnew +setlocal\ bufhidden=wipe
tabrewind
edit src/arduinoMI/Peaches_Braids/Peaches_Braids.ino
argglobal
setlocal fdm=syntax
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
106
normal! zo
109
normal! zo
110
normal! zo
130
normal! zo
138
normal! zo
147
normal! zo
187
normal! zo
226
normal! zo
228
normal! zo
237
normal! zo
245
normal! zo
265
normal! zo
268
normal! zo
280
normal! zo
282
normal! zo
289
normal! zo
300
normal! zo
288
normal! zo
300
normal! zo
302
normal! zo
let s:l = 238 - ((8 * winheight(0) + 25) / 51)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 238
normal! 057|
lcd ~/src/arduinoMI/Peaches_Braids
tabnext
edit ~/src/arduinoMI/Peaches_Braids/braids.h
argglobal
balt ~/src/arduinoMI/Peaches_Braids/Peaches_Braids.ino
setlocal fdm=syntax
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
74
normal! zo
112
normal! zo
119
normal! zo
143
normal! zo
157
normal! zo
191
normal! zo
let s:l = 148 - ((12 * winheight(0) + 25) / 51)
if s:l < 1 | let s:l = 1 | endif
keepjumps exe s:l
normal! zt
keepjumps 148
normal! 06|
lcd ~/src/arduinoMI/Peaches_Braids
tabnext 1
set stal=1
if exists('s:wipebuf') && len(win_findbuf(s:wipebuf)) == 0
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20
let &shortmess = s:shortmess_save
let s:sx = expand("<sfile>:p:r")."x.vim"
if filereadable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &g:so = s:so_save | let &g:siso = s:siso_save
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
