\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{WHUthesis}
[2011/03/15 v0.1j WHUthesis document class]
\newif\ifWHUT@typeinfo \WHUT@typeinfotrue   % 声明一个参数\WHUT@typeinfo，并设置该参数为true
\newif\ifWHUT@dvips \WHUT@dvipsfalse        % 声明一个参数\WHUT@dvipsfalse，并设置该参数为false
% ---------------------------------------------------------------
% 参数说明
\DeclareOption{notypeinfo}{\WHUT@typeinfofalse}
\DeclareOption{dvips}{\WHUT@dvipstrue} % dvips
\DeclareOption{dvipdfm}{\WHUT@dvipsfalse} %
\DeclareOption{pdftex}{\WHUT@dvipsfalse}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{ctexbook}}
\ProcessOptions
% ----------------------------------------------------------------
% 载入需要的class
\LoadClass[CJK,cs4size,a4paper,fancyhdr,fntef]{ctexbook}[2005/11/25]
% ----------------------------------------------------------------
% 载入需要的packages
\RequirePackage{ifpdf} %% check pdfTeX mode
\RequirePackage{amsmath,amsthm,amsfonts,amssymb,bm,cmap} %% math packages
\RequirePackage{multirow}
\RequirePackage{graphicx}%% graphics packages
\RequirePackage[sort&compress,square,comma,numbers]{natbib}%% sort and compress citations
%% packages and settings depend on PDF mode
\ifpdf   % We're running pdfTeX in PDF mode
  \RequirePackage[pdftex]{hyperref}
  \DeclareGraphicsExtensions{.pdf}
\else    % We're not running pdfTeX, or running pdfTeX in DVI mode
  \ifWHUT@dvips
    \RequirePackage[dvips]{hyperref}
    \RequirePackage{psfrag}
    \AtBeginDvi{\special{pdf:tounicode GBK-EUC-UCS2}} % GBK -> Unicode
  \else
    \RequirePackage[dvipdfm]{hyperref}
  \fi
  \DeclareGraphicsExtensions{.eps,.ps}
\fi
% -------------------------------------------------------------------
\hypersetup{CJKbookmarks,%
       bookmarksnumbered,%
              colorlinks,%
               linkcolor=blue,%
               citecolor=blue,%
              plainpages=false,%
            pdfstartview=FitH}
\AtEndOfPackage{\makeatletter\input{WHUthesis.cfg}\makeatother}
% --------------------------------------------------------------------
% 设置图和表的Caption的格式
\CTEXoptions[captiondelimiter={~~}]
\def\CTEX@figurename{\songti\zihao{5}图}
\def\CTEX@tablename{\songti\zihao{5}表}
% --------------------------------------------------------------------
% 页面设置
%\oddsidemargin 1 true cm          %左边边距大小
%\if@twoside                         %如果双面打印，偶数页边距设置
%  \evensidemargin 0 true cm
%\else
%  \evensidemargin 1 true cm
%\fi
%\textheight 21 true cm              % 文本区域高度
%\textwidth 14.5 true cm             % 文本区域宽度
%\headheight 0pt                     % 页眉高度
%\footskip 40pt
%\parskip 0.5ex plus 0.25ex minus 0.25ex
\RequirePackage[left=3truecm,right=3truecm,top=3truecm,bottom=3.3truecm]{geometry}
\footskip 30pt
\headheight 0pt
\headsep 20pt
\topmargin 0pt
\oddsidemargin 1.5 true cm
\if@twoside                         %如果双面打印，偶数页边距设置
  \evensidemargin -0.6 true cm
\else
  \evensidemargin 0.0 true cm
\fi
\renewcommand\baselinestretch{1.38}

% 如果章节的结束页面是奇数页，则新加一页
\def\cleardoublepage{\clearpage\if@twoside \ifodd\c@page\else
  \thispagestyle{plain}%
  \hbox{}\newpage\if@twocolumn\hbox{}\newpage\fi\fi\fi}
\def\cleardoublepagecover{\clearpage\if@twoside \ifodd\c@page\else
  \thispagestyle{empty}%
  \hbox{}\newpage\if@twocolumn\hbox{}\newpage\fi\fi\fi}

\renewcommand{\floatpagefraction}{0.80} % 浮动页面上浮动对象占用的最小页面比例
% ---------------------------------------------------------------------------------
% -------------------------------------------------------------------------------
% 乱七八糟的定义
\newcommand\WHUTspace{\protect\CTEX@spaceChar\protect\CTEX@spaceChar}
\newcommand\todaybig{\CTEXdigits{\yearbig}{\the\year}\yearbig 年 \CTEXnumber{\monthbig}{\the\month}\monthbig 月}
\newcommand\todayold{\CTEXoptions[today=old]\today}
\newcommand\bodyformat{\linespread{1.5}\selectfont\songti\zihao{-4}}
% -------------------------------------------------------------------------
\def\WHUT@getfileinfo#1 #2 #3\relax#4\relax{%
  \def\WHUTfiledate{#1}%
  \def\WHUTfileversion{#2}%
  \def\WHUTfileinfo{#3}}%
\expandafter\ifx\csname ver@WHUthesis.cls\endcsname\relax
  \edef\reserved@a{\csname ver@ctextemp_WHUthesis.cls\endcsname}
\else
  \edef\reserved@a{\csname ver@WHUthesis.cls\endcsname}
\fi
\expandafter\WHUT@getfileinfo\reserved@a\relax\relax\relax
% -------------------------------------------------------------------------
\def\WHUT@underline[#1]#2{%
  \CTEXunderline{\hbox to #1{\hfill#2\hfill}}}
\def\WHUTunderline{\@ifnextchar[\WHUT@underline\CTEXunderline}
% -------------------------------------------------------------------------
% 定义论文的基本信息：分类，密级，UDC，序列号，标题，作者等等
\newcommand\classification[1]{\def\WHUT@value@classification{#1}}   % 分类号
\newcommand\confidential[1]{\def\WHUT@value@confidential{#1}}       % 密级
\newcommand\UDC[1]{\def\WHUT@value@UDC{#1}}                         % UDC
\newcommand\serialnumber[1]{\def\WHUT@value@serialnumber{#1}}       % 编号
\renewcommand\title[2][\WHUT@value@title]{%                         % 文章标题（及缩写）
  \def\WHUT@value@title{#2}
  \def\WHUT@value@titlemark{\MakeUppercase{#1}}}
\renewcommand\author[1]{\def\WHUT@value@author{#1}}                 % 作者
\newcommand\institute[1]{\def\WHUT@value@institute{#1}}             % 作者单位
\newcommand\advisor[1]{\def\WHUT@value@advisor{#1}}                 % 指导老师
\newcommand\advisortitle[1]{\def\WHUT@value@advisortitle{#1}}       % 指导老师的职称
\newcommand\advisorinstitute[1]{\def\WHUT@value@advisorinstitute{#1}}% 指导老师单位
\newcommand\major[1]{\def\WHUT@value@major{#1}}                     % 专业
\newcommand\field[1]{\def\WHUT@value@field{#1}}                     % 研究方向
\newcommand\defenddate[1]{\def\WHUT@value@defenddate{#1}}           % 答辩日期
\newcommand\degree[1]{\def\WHUT@value@degree{#1}}                   % 申请学位类型
% -------------------------------------------------------------------------------------------
% 论文的英文版信息
\newcommand\englishtitle[1]{\def\WHUT@value@englishtitle{#1}}       % 英文标题
\newcommand\englishauthor[1]{\def\WHUT@value@englishauthor{#1}}     % 作者英文名
\newcommand\englishadvisor[1]{\def\WHUT@value@englishadvisor{#1}}   % 导师的英文名
\newcommand\englishinstitute[1]{\def\WHUT@value@englishinstitute{#1}} % 导师单位的英文名称
\newcommand\englishdate[1]{\def\WHUT@value@englishdate{#1}}         % 答辩日期英文
\newcommand\englishdegree[1]{\def\WHUT@value@englishdegree{#1}}     % 申请学位类型
\newcommand\englishmajor[1]{\def\WHUT@value@englishmajor{#1}}       % 专业英文
% -------------------------------------------------------------------------------------------
% 中文封面
\renewcommand\maketitle{%
  \pdfbookmark[0]{\WHUT@value@title}{title}
  \cleardoublepagecover
  \thispagestyle{empty}
  \begin{center}
% -------------------------------------------------------------------------------------------
    %\vskip 25mm
    \def\topformat{\fangsong\zihao{-4}}
    \makebox[3.5em][s]{\topformat\WHUT@label@classification}~\WHUTunderline[80pt]{\topformat\WHUT@value@classification}\hfill
    \makebox[3.5em][s]{\topformat\WHUT@label@confidential}~\WHUTunderline[80pt]{\topformat\WHUT@value@confidential}\\
    \makebox[3.5em][s]{\topformat\WHUT@label@UDC}~\WHUTunderline[80pt]{\topformat\WHUT@value@UDC}\hfill
    \makebox[3.5em][s]{\topformat\WHUT@label@serialnumber}~\WHUTunderline[80pt]{\topformat\WHUT@value@serialnumber}
% -------------------------------------------------------------------------------------------
    \vskip 20mm
    \def\institueformat{\songti\zihao{2}}
    \makebox[14em][s]{\institueformat\WHUT@value@school}\\
    \vskip 1em
    \makebox[22em][s]{\institueformat\WHUT@value@degree\WHUT@label@thesis}
% -------------------------------------------------------------------------------------------
    \vskip 25mm
    \def\titleformat{\large\heiti\zihao{1}}
    {\linespread{1}\selectfont\titleformat\WHUT@value@title}
    \vfill
    \linespread{2}\selectfont
    \def\inforformat{\songti\zihao{4}}
    \def\tabcolsep{1pt}
    \def\arraystretch{1.2}
    \begin{tabular}{ll}%
        \makebox[12em][s]{\inforformat\WHUT@label@author}~：&\hspace{0.5em} {\inforformat\WHUT@value@author}%
        \\
        \makebox[12em][s]{\inforformat\WHUT@label@advisor}~：&\hspace{.5em} {\inforformat\WHUT@value@advisor\quad\WHUT@value@advisortitle}%
        \\
        %%***如果要加入副导师姓名，就不要注释下面的步骤
        %%& \hspace{.5em} {\inforformat\WHUT@value@coadvisor\quad\WHUT@value@coadvisortitle}%
        %%\\
        \makebox[12em][s]{\inforformat\WHUT@label@major}~：&\hspace{.5em} {\inforformat\WHUT@value@major}%
        \\
        %\makebox[12em][s]{\inforformat\WHUT@label@field}~：&\hspace{.5em} {\inforformat\WHUT@value@field}%
    \end{tabular}%
    \vskip 25mm
    \def\bottomformat{\heiti\zihao{3}}
    \makebox[\textwidth][c]{\bottomformat\WHUT@value@defenddate}%
    \vskip 10mm%
  \end{center}
  \clearpage
  \if@twoside
    \thispagestyle{empty}
    \ifWHUT@typeinfo
      \vspace*{\stretch{1}}
      \begin{footnotesize}
        \noindent
        Typeset by \LaTeXe{} at \CTEX@todayold \\
        With package \texttt{WHUthesis} \WHUTfileversion{} of C\TeX{}.ORG
      \end{footnotesize}
    \fi
    \cleardoublepagecover
    \thispagestyle{empty}
  \fi
}
%---------------------------------------------------------------------------
% 英文封面
\newcommand\makeenglishtitle{%
  \cleardoublepagecover
  \thispagestyle{empty}
  \begin{center}
    \vspace*{20pt}
      \sf\zihao{-1} \WHUT@value@englishtitle
    \vskip \stretch{1}
      \bf\zihao{4} \WHUT@value@englishauthor
    \vskip \stretch{1}
      \normalfont\zihao{4} \WHUT@label@englishadvisor
    \vskip 3pt
      \normalfont\zihao{4} \WHUT@value@englishadvisor
    \vskip \stretch{1}
      \includegraphics[width=.30\textwidth]{cover/xidian_logo.pdf}
    \vskip \stretch{1}
      \normalfont\normalsize \WHUT@value@englishinstitute
    \vskip 30pt
      \normalfont\normalsize \WHUT@value@englishdate
    \vskip 20pt
      \it\normalsize \WHUT@label@englishstatement
  \end{center}
  \clearpage
  \if@twoside
    \thispagestyle{empty}
    \cleardoublepagecover
    \thispagestyle{empty}
  \fi
}

% ---------------------------------------------------------------------
%% 郑重声明页
%%
\newcommand\makeclaim{
  \chapter*{\WHUT@value@claimtitle}
  \vskip 15mm
  \linespread{2}\selectfont\songti\zihao{-4}\WHUT@value@claim
  \vskip 20mm
  \linespread{1.4}\selectfont%正文行距应该设置为1.4
  \hfill\WHUT@value@claimsign
  \vskip 1em
  \hfill\WHUT@value@claimdate
  \thispagestyle{empty}
  \cleardoublepagecover
}
% --------------------------------------------------------------------
%% 学位论文使用授权书
%%
\newcommand\makeauth{
\chapter*{\WHUT@value@authtitle}
  \vskip 15mm
  \linespread{2}\selectfont\songti\zihao{-4}\WHUT@value@auth
  \thispagestyle{empty}
  \cleardoublepagecover
}
% ---------------------------------------------------------------
%% 创新点
\newenvironment{novelty}{
  \chapter*{\WHUT@value@noveltitle}
  \thispagestyle{empty}
}{\thispagestyle{empty}
  \cleardoublepagecover
  \setcounter{page}{1}}

%% 页眉页脚设置
\fancypagestyle{plain}{   %章页中的设置
  \fancyhf{}%
\fancyhead[OC]{\zihao{5}\songti\leftmark}
\fancyhead[OR]{\zihao{-5}\thepage}
\fancyhead[EC]{\zihao{5}\songti\leftmark}
\fancyhead[EL]{\zihao{-5}\thepage}
\renewcommand{\headrulewidth}{0.75pt} %分隔线
\renewcommand{\footrulewidth}{0pt}
}

\pagestyle{fancy}{ %正文中的设置
    \fancyhf{}
\fancyhead[OC]{\zihao{5}\songti\leftmark} %奇数页，章标题
\fancyhead[OR]{\zihao{-5}\thepage}
\fancyhead[EC]{\zihao{5}\songti\WHUT@value@title} %论文题目
\fancyhead[EL]{\zihao{-5}\thepage}
\renewcommand\headrulewidth{0.75pt}
\renewcommand{\footrulewidth}{0pt}
}
%% 摘要设置
% 中文摘要
\newenvironment{abstract}
  {\Nchapter{\WHUT@label@abstract}}
  {}
\newcommand\keywords[1]{%
  \vspace{2em}\noindent{\heiti \WHUT@label@keywords} #1}
% 英文摘要
\newenvironment{englishabstract}
  {\Nchapter{\bf\WHUT@label@englishabstract}}
  {}
\newcommand\englishkeywords[1]{%
  \vspace{2em}\noindent{\bf \WHUT@label@englishkeywords} #1}
% 生成目录
\renewcommand\tableofcontents{%
    \if@twocolumn
      \@restonecoltrue\onecolumn
    \else
      \@restonecolfalse
    \fi
    \Nchapter{\contentsname}%
    \@mkboth{\MakeUppercase\contentsname}{\MakeUppercase\contentsname}%
    \@starttoc{toc}%
    \if@restonecol\twocolumn\fi
    }
\addtocontents{toc}{\let\string\CTEX@spaceChar\relax}
\newcommand\Nchapter[1]{%
  \if@mainmatter%
    \@mainmatterfalse%
    \chapter{#1}%
    \@mainmattertrue%
  \else
    \chapter{#1}%
  \fi}
\def\@dottedtocline#1#2#3#4#5{%
  \ifnum #1>\c@tocdepth \else
    \vskip \z@ \@plus.2\p@
    {\leftskip #2\relax \rightskip \@tocrmarg \parfillskip -\rightskip
     \parindent #2\relax\@afterindenttrue
     \interlinepenalty\@M
     \leavevmode
     \@tempdima #3\relax
     \advance\leftskip \@tempdima \null\nobreak\hskip -\leftskip
     {#4}\nobreak
     \leaders\hbox{$\m@th\mkern 1.5mu\cdot\mkern 1.5mu$}\hfill
     \nobreak
     \hb@xt@\@pnumwidth{\hfil\normalfont \normalcolor #5}%
     \par}%
  \fi}
\renewcommand*\l@part[2]{%
  \ifnum \c@tocdepth >-2\relax
    \addpenalty{-\@highpenalty}%
    \addvspace{2.25em \@plus\p@}%
    \setlength\@tempdima{3em}%
    \begingroup
      \parindent \z@ \rightskip \@pnumwidth
      \parfillskip -\@pnumwidth
      {\leavevmode
       \large \bfseries #1
       \leaders\hbox{$\m@th\mkern 1.5mu\cdot\mkern 1.5mu$}
       \hfil \hb@xt@\@pnumwidth{\hss #2}}\par
       \nobreak
         \global\@nobreaktrue
         \everypar{\global\@nobreakfalse\everypar{}}%
    \endgroup
  \fi}
\renewcommand*\l@chapter[2]{%
  \ifnum \c@tocdepth >\m@ne
    \addpenalty{-\@highpenalty}%
    \vskip 0.1ex \@plus\p@
    \setlength\@tempdima{1.5em}%
    \begingroup
      \parindent \z@ \rightskip \@pnumwidth
      \parfillskip -\@pnumwidth
      \leavevmode \bfseries
      \advance\leftskip\@tempdima
      \hskip -\leftskip
      #1\nobreak
      \leaders\hbox{$\m@th\mkern 1.5mu\cdot\mkern 1.5mu$}
      \hfil \nobreak\hb@xt@\@pnumwidth{\hss #2}\par
      \penalty\@highpenalty
    \endgroup
  \fi}
%% 参考文献设置
\renewenvironment{thebibliography}[1]
     {\Nchapter{\bibname}%
      \@mkboth{\MakeUppercase\bibname}{\MakeUppercase\bibname}%
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\@arabic\c@enumiv}}%
      \small
      \sloppy
      \clubpenalty4000
      \@clubpenalty \clubpenalty
      \widowpenalty4000%
      \sfcode`\.\@m}
     {\def\@noitemerr
       {\@latex@warning{Empty `thebibliography' environment}}%
      \endlist}
%  发表文章
\newenvironment{publications}[1]
     {\Nchapter{\WHUT@label@publications}%
      \@mkboth{\MakeUppercase\WHUT@label@publications}
              {\MakeUppercase\WHUT@label@publications}%
      \list{\@biblabel{\@arabic\c@enumiv}}%
           {\settowidth\labelwidth{\@biblabel{#1}}%d
            \leftmargin\labelwidth
            \advance\leftmargin\labelsep
            \@openbib@code
            \usecounter{enumiv}%
            \let\p@enumiv\@empty
            \renewcommand\theenumiv{\@arabic\c@enumiv}}%
      \sloppy
      \clubpenalty4000
      \@clubpenalty \clubpenalty
      \widowpenalty4000%
      \sfcode`\.\@m}
     {\def\@noitemerr
       {\@latex@warning{Empty `publications' environment}}%
      \endlist}
%% 简历页面设置
\newenvironment{resume}
  {\Nchapter{\WHUT@label@resume}}
  {}
\newenvironment{resumesection}[1]
  {{\noindent\normalfont\bfseries #1}
   \list{}{\labelwidth\z@
           \leftmargin 2\ccwd}
   \item\relax}
  {\endlist}
\newenvironment{resumelist}[1]
  {{\noindent\normalfont\bfseries #1}
   \list{}{\labelwidth\z@
           \leftmargin 4\ccwd
           \itemindent -2\ccwd
           \listparindent\itemindent}
   \item\relax}
  {\endlist}

%% 致谢部分
\renewenvironment{thanks}
  {\Nchapter{\WHUT@label@thanks}}
  {}
\setcounter{secnumdepth}{4}
\newtheoremstyle{break}% name
  {}%      Space above, empty = `usual value'
  {}%      Space below
  {\itshape}% Body font
  {}%         Indent amount (empty = no indent, \parindent = para indent)
  {\bfseries}% Thm head font
  {.}%        Punctuation after thm head
  {\newline}% Space after thm head: \newline = linebreak
  {}%         Thm head spec
\endinput
%%
%% End of file `WHUthesis.cls'.
