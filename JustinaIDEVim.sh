sudo apt-get install vim-nox-py2

curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
git clone https://github.com/VundleVim/Vundle.vim.git ~/.vim/bundle/Vundle.vim

echo "set nocompatible" >> ~/.vimrc
echo "filetype off" >> ~/.vimrc
echo "set rtp+=~/.vim/bundle/Vundle.vim" >> ~/.vimrc
echo "call vundle#begin()" >> ~/.vimrc
echo "Plugin 'VundleVim/Vundle.vim'" >> ~/.vimrc
echo "Plugin 'Valloric/YouCompleteMe'" >> ~/.vimrc
echo "Plugin 'https://github.com/taketwo/vim-ros'" >> ~/.vimrc
echo "call vundle#end()" >> ~/.vimrc
echo "filetype plugin indent on" >> ~/.vimrc
echo "let g:ycm_semantic_triggers = {" >> ~/.vimrc
echo "                        \   'roslaunch' : ['=\"', '\$(', '/']," >> ~/.vimrc
echo "                        \   'rosmsg,rossrv,rosaction' : ['re!^', '/']," >> ~/.vimrc
echo "                        \ }" >> ~/.vimrc
echo "function! BuildYCM(info)" >> ~/.vimrc
echo "        if a:info.status == 'installed' || a:info.force" >> ~/.vimrc
echo "                !./install.py" >> ~/.vimrc
echo "        endif" >>  ~/.vimrc
echo "endfunction" >> ~/.vimrc
echo "call plug#begin('~/.vim/plugged')" >> ~/.vimrc
echo "Plug 'https://github.com/taketwo/vim-ros'" >> ~/.vimrc
echo "Plug 'Valloric/YouCompleteMe', { 'do': function('BuildYCM') }" >> ~/.vimrc
echo "call plug#end()" >> ~/.vimrc
echo "autocmd FileType cpp setlocal number" >> ~/.vimrc
echo "autocmd FileType cpp setlocal expandtab shiftwidth=4 softtabstop=4 tabstop=4 smarttab" >> ~/.vimrc

vim +PlugInstall +qall

cd ~/.vim/plugged/YouCompleteMe
./install.py --clang-completer


