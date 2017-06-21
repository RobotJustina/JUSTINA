colorscheme delek       " awesome colorscheme
syntax enable           " enable syntax processing
set tabstop=4
set softtabstop=4   " number of spaces in tab when editing
set expandtab       " tabs are spaces
set number              " show line numbers
set showcmd             " show command in bottom bar
set cursorline          " highlight current line
filetype indent on      " load filetype-specific indent files
set wildmenu            " visual autocomplete for command menu
set showmatch           " highlight matching [{()}]
set incsearch           " search as characters are entered
set hlsearch            " highlight matches

noremap <leader><space> :nohlsearch<CR>

set foldenable          " enable folding
set foldlevelstart=20   " open most folds by default
set foldnestmax=10      " 10 nested fold max
set foldmethod=syntax   " fold based on indent level
nnoremap <space> zA

let mapleader=","       " leader is comma
