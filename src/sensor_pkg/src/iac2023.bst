%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is a bib compiler for IAC.
% This file is based on ieeetran.bst.
% https://ftp.kddilabs.jp/CTAN/macros/latex/contrib/IEEEtran/bibtex/IEEEtran.bst
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% #0 turns off the display of the number for articles.
% #1 enables
FUNCTION {default.is.use.number.for.article} { #1 }

% #0 turns off the display of the paper and type fields in @inproceedings.
% #1 enables

FUNCTION {default.is.use.paper} { #1 }
% #0 turns off the display of urls
% #1 enables
FUNCTION {default.is.use.url} { #1 }

% #0 turns off the forced use of "et al."
% #1 enables
FUNCTION {default.is.forced.et.al} { #0 }

% The maximum number of names that can be present beyond which an "et al."
% usage is forced. Be sure that num.names.shown.with.forced.et.al (below)
% is not greater than this value!
% Note: There are many instances of references in IEEE journals which have
% a very large number of authors as well as instances in which "et al." is
% used profusely.
FUNCTION {default.max.num.names.before.forced.et.al} { #10 }

% The number of names that will be shown with a forced "et al.".
% Must be less than or equal to max.num.names.before.forced.et.al
FUNCTION {default.num.names.shown.with.forced.et.al} { #1 }

% #0 turns off the alternate interword spacing for entries with URLs.
% #1 enables
FUNCTION {default.is.use.alt.interword.spacing} { #1 }

% If alternate interword spacing for entries with URLs is enabled, this is
% the interword spacing stretch factor that will be used. For example, the
% default "4" here means that the interword spacing in entries with URLs can
% stretch to four times normal. Does not have to be an integer. Note that
% the value specified here can be overridden by the user in their LaTeX
% code via a command such as: 
% "\providecommand\BIBentryALTinterwordstretchfactor{1.5}" in addition to
% that via the IEEEtranBSTCTL entry type.
FUNCTION {default.ALTinterwordstretchfactor} { "4" }


% #0 turns off the "dashification" of repeated (i.e., identical to those
% of the previous entry) names. The IEEE normally does this.
% #1 enables
FUNCTION {default.is.dash.repeated.names} { #1 }


% The default name format control string.
FUNCTION {default.name.format.string}{ "{f{.}.~}{vv~}{ll}{ jj}" }


% The default LaTeX font command for the names.
FUNCTION {default.name.latex.cmd}{ "" }


% The default URL prefix.
FUNCTION {default.name.url.prefix}{ "" }


% Other controls that cannot be accessed via IEEEtranBSTCTL entry type.

% #0 turns off the terminal startup banner/completed message so as to
% operate more quietly.
% #1 enables
FUNCTION {is.print.banners.to.terminal} { #1 }


%%%%%%%%%%%%%%%%%%%%%%
%% STRING CONSTANTS %%
%%%%%%%%%%%%%%%%%%%%%%

% FUNCTION {bbl.and}{ "and" } % 著者が二人以上いる場合，A. Mike, B. Jon and C. Kenになる．
FUNCTION {bbl.etal}{ "et~al." }
FUNCTION {bbl.editors}{ "eds." }
FUNCTION {bbl.editor}{ "ed." }
FUNCTION {bbl.edition}{ "ed." }
FUNCTION {bbl.volume}{ "vol." }
FUNCTION {bbl.of}{ "of" }
FUNCTION {bbl.number}{ "no." }
FUNCTION {bbl.in}{ "in" }
FUNCTION {bbl.pages}{ "pp." }
FUNCTION {bbl.page}{ "p." }
FUNCTION {bbl.chapter}{ "ch." }
FUNCTION {bbl.paper}{ "paper" }
FUNCTION {bbl.part}{ "pt." }
FUNCTION {bbl.patent}{ "Patent" }
FUNCTION {bbl.patentUS}{ "U.S." }
FUNCTION {bbl.revision}{ "Rev." }
FUNCTION {bbl.series}{ "ser." }
FUNCTION {bbl.standard}{ "Std." }
FUNCTION {bbl.techrep}{ "Tech. Rep." }
FUNCTION {bbl.mthesis}{ "Master's thesis" }
FUNCTION {bbl.phdthesis}{ "Ph.D. dissertation" }
FUNCTION {bbl.st}{ "st" }
FUNCTION {bbl.nd}{ "nd" }
FUNCTION {bbl.rd}{ "rd" }
FUNCTION {bbl.th}{ "th" }


% This is the LaTeX spacer that is used when a larger than normal space
% is called for (such as just before the address:publisher).
FUNCTION {large.space} { "\hskip 1em plus 0.5em minus 0.4em\relax " }

% The LaTeX code for dashes that are used to represent repeated names.
% Note: Some older IEEE journals used something like
% "\rule{0.275in}{0.5pt}\," which is fairly thick and runs right along
% the baseline. However, the IEEE now uses a thinner, above baseline,
% six dash long sequence.
FUNCTION {repeated.name.dashes} { "------" }



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PREDEFINED STRING MACROS %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

MACRO {jan} {"Jan."}
MACRO {feb} {"Feb."}
MACRO {mar} {"Mar."}
MACRO {apr} {"Apr."}
MACRO {may} {"May"}
MACRO {jun} {"Jun."}
MACRO {jul} {"Jul."}
MACRO {aug} {"Aug."}
MACRO {sep} {"Sep."}
MACRO {oct} {"Oct."}
MACRO {nov} {"Nov."}
MACRO {dec} {"Dec."}



%%%%%%%%%%%%%%%%%%
%% ENTRY FIELDS %%
%%%%%%%%%%%%%%%%%%

ENTRY
  { address
    assignee
    author
    booktitle
    chapter
    day
    dayfiled
    edition
    editor
    howpublished
    institution
    intype
    journal
    key
    language
    month
    monthfiled
    nationality
    note
    number
    organization
    pages
    paper
    publisher
    school
    series
    revision
    title
    type
    url
    volume
    year
    yearfiled
    CTLuse_article_number
    CTLuse_paper
    CTLuse_url
    CTLuse_forced_etal
    CTLmax_names_forced_etal
    CTLnames_show_etal
    CTLuse_alt_spacing
    CTLalt_stretch_factor
    CTLdash_repeated_names
    CTLname_format_string
    CTLname_latex_cmd
    CTLname_url_prefix
  }
  {}
  { label }



%%%%%%%%%%%%%%%%%%%%%%%
%% INTEGER VARIABLES %%
%%%%%%%%%%%%%%%%%%%%%%%

INTEGERS { prev.status.punct this.status.punct punct.std
           punct.no punct.comma punct.period 
           prev.status.space this.status.space space.std
           space.no space.normal space.large
           prev.status.quote this.status.quote quote.std
           quote.no quote.close
           prev.status.nline this.status.nline nline.std
           nline.no nline.newblock 
           status.cap cap.std
           cap.no cap.yes}

INTEGERS { longest.label.width multiresult nameptr namesleft number.label numnames }

INTEGERS { is.use.number.for.article
           is.use.paper
           is.use.url
           is.forced.et.al
           max.num.names.before.forced.et.al
           num.names.shown.with.forced.et.al
           is.use.alt.interword.spacing
           is.dash.repeated.names}


%%%%%%%%%%%%%%%%%%%%%%
%% STRING VARIABLES %%
%%%%%%%%%%%%%%%%%%%%%%

STRINGS { bibinfo
          longest.label
          oldname
          s
          t
          ALTinterwordstretchfactor
          name.format.string
          name.latex.cmd
          name.url.prefix}

%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOW LEVEL FUNCTIONS %%
%%%%%%%%%%%%%%%%%%%%%%%%%

FUNCTION {initialize.controls}
{ default.is.use.number.for.article 'is.use.number.for.article :=
  default.is.use.paper 'is.use.paper :=
  default.is.use.url 'is.use.url :=
  default.is.forced.et.al 'is.forced.et.al :=
  default.max.num.names.before.forced.et.al 'max.num.names.before.forced.et.al :=
  default.num.names.shown.with.forced.et.al 'num.names.shown.with.forced.et.al :=
  default.is.use.alt.interword.spacing 'is.use.alt.interword.spacing :=
  default.is.dash.repeated.names 'is.dash.repeated.names :=
  default.ALTinterwordstretchfactor 'ALTinterwordstretchfactor :=
  default.name.format.string 'name.format.string :=
  default.name.latex.cmd 'name.latex.cmd :=
  default.name.url.prefix 'name.url.prefix :=
}

FUNCTION {initialize.status.constants}
{ #0 'punct.no :=
  #1 'punct.comma :=
  #2 'punct.period :=
  #0 'space.no := 
  #1 'space.normal :=
  #2 'space.large :=
  #0 'quote.no :=
  #1 'quote.close :=
  #0 'cap.no :=
  #1 'cap.yes :=
  #0 'nline.no :=
  #1 'nline.newblock :=
}

FUNCTION {std.status.using.comma}
{ punct.comma 'punct.std :=
  space.normal 'space.std :=
  quote.no 'quote.std :=
  nline.no 'nline.std :=
  cap.no 'cap.std :=
}

FUNCTION {std.status.using.period}
{ punct.period 'punct.std :=
  space.normal 'space.std :=
  quote.no 'quote.std :=
  nline.no 'nline.std :=
  cap.yes 'cap.std :=
}

FUNCTION {initialize.prev.this.status}
{ punct.no 'prev.status.punct :=
  space.no 'prev.status.space :=
  quote.no 'prev.status.quote :=
  nline.no 'prev.status.nline :=
  punct.no 'this.status.punct :=
  space.no 'this.status.space :=
  quote.no 'this.status.quote :=
  nline.no 'this.status.nline :=
  cap.yes 'status.cap :=
}

FUNCTION {this.status.std}
{ punct.std 'this.status.punct :=
  space.std 'this.status.space :=
  quote.std 'this.status.quote :=
  nline.std 'this.status.nline :=
}

FUNCTION {cap.status.std}{ cap.std 'status.cap := }

FUNCTION {this.to.prev.status}
{ this.status.punct 'prev.status.punct :=
  this.status.space 'prev.status.space :=
  this.status.quote 'prev.status.quote :=
  this.status.nline 'prev.status.nline :=
}


FUNCTION {not}
{   { #0 }
    { #1 }
  if$
}

FUNCTION {and}
{   { skip$ }
    { pop$ #0 }
  if$
}

FUNCTION {or}
{   { pop$ #1 }
    { skip$ }
  if$
}


% convert the strings "yes" or "no" to #1 or #0 respectively
FUNCTION {yes.no.to.int}
{ "l" change.case$ duplicate$
    "yes" =
    { pop$  #1 }
    { duplicate$ "no" =
        { pop$ #0 }
        { "unknown boolean " quote$ * swap$ * quote$ *
          " in " * cite$ * warning$
          #0
        }
      if$
    }
  if$
}


% pushes true if the single char string on the stack is in the
% range of "0" to "9"
FUNCTION {is.num}
{ chr.to.int$
  duplicate$ "0" chr.to.int$ < not
  swap$ "9" chr.to.int$ > not and
}

% multiplies the integer on the stack by a factor of 10
FUNCTION {bump.int.mag}
{ #0 'multiresult :=
    { duplicate$ #0 > }
    { #1 -
      multiresult #10 +
      'multiresult :=
    }
  while$
pop$
multiresult
}

% converts a single character string on the stack to an integer
FUNCTION {char.to.integer}
{ duplicate$ 
  is.num
    { chr.to.int$ "0" chr.to.int$ - }
    {"noninteger character " quote$ * swap$ * quote$ *
          " in integer field of " * cite$ * warning$
    #0
    }
  if$
}

% converts a string on the stack to an integer
FUNCTION {string.to.integer}
{ duplicate$ text.length$ 'namesleft :=
  #1 'nameptr :=
  #0 'numnames :=
    { nameptr namesleft > not }
    { duplicate$ nameptr #1 substring$
      char.to.integer numnames bump.int.mag +
      'numnames :=
      nameptr #1 +
      'nameptr :=
    }
  while$
pop$
numnames
}




% The output routines write out the *next* to the top (previous) item on the
% stack, adding punctuation and such as needed. Since IEEEtran.bst maintains
% the output status for the top two items on the stack, these output
% routines have to consider the previous output status (which corresponds to
% the item that is being output). Full independent control of punctuation,
% closing quote marks, spacing, and newblock is provided.
% 
% "output.nonnull" does not check for the presence of a previous empty
% item.
% 
% "output" does check for the presence of a previous empty item and will
% remove an empty item rather than outputing it.
% 
% "output.warn" is like "output", but will issue a warning if it detects
% an empty item.

FUNCTION {output.nonnull}
{ swap$
  prev.status.punct punct.comma =
     { "," * }
     { skip$ }
   if$
  prev.status.punct punct.period =
     { add.period$ }
     { skip$ }
   if$ 
  prev.status.quote quote.close =
     { "''" * }
     { skip$ }
   if$
  prev.status.space space.normal =
     { " " * }
     { skip$ }
   if$
  prev.status.space space.large =
     { large.space * }
     { skip$ }
   if$
  write$
  prev.status.nline nline.newblock =
     { newline$ "\newblock " write$ }
     { skip$ }
   if$
}

FUNCTION {output}
{ duplicate$ empty$
    'pop$
    'output.nonnull
  if$
}

FUNCTION {output.warn}
{ 't :=
  duplicate$ empty$
    { pop$ "empty " t * " in " * cite$ * warning$ }
    'output.nonnull
  if$
}

% "fin.entry" is the output routine that handles the last item of the entry
% (which will be on the top of the stack when "fin.entry" is called).

FUNCTION {fin.entry}
{ this.status.punct punct.no =
     { skip$ }
     { add.period$ }
   if$
   this.status.quote quote.close =
     { "''" * }
     { skip$ }
   if$
write$
newline$
}


FUNCTION {is.last.char.not.punct}
{ duplicate$
   "}" * add.period$
   #-1 #1 substring$ "." =
}

FUNCTION {is.multiple.pages}
{ 't :=
  #0 'multiresult :=
    { multiresult not
      t empty$ not
      and
    }
    { t #1 #1 substring$
      duplicate$ "-" =
      swap$ duplicate$ "," =
      swap$ "+" =
      or or
        { #1 'multiresult := }
        { t #2 global.max$ substring$ 't := }
      if$
    }
  while$
  multiresult
}

FUNCTION {capitalize}{ "u" change.case$ "t" change.case$ }

FUNCTION {emphasize}
{ duplicate$ empty$
    { pop$ "" }
    { "\emph{" swap$ * "}" * }
  if$
}

FUNCTION {do.name.latex.cmd}
{ name.latex.cmd
  empty$
    { skip$ }
    { name.latex.cmd "{" * swap$ * "}" * }
  if$
}

% IEEEtran.bst uses its own \BIBforeignlanguage command which directly
% invokes the TeX hyphenation patterns without the need of the Babel
% package. Babel does a lot more than switch hyphenation patterns and
% its loading can cause unintended effects in many class files (such as
% IEEEtran.cls).
FUNCTION {select.language}
{ duplicate$ empty$ 'pop$
    { language empty$ 'skip$
        { "\BIBforeignlanguage{" language * "}{" * swap$ * "}" * }
      if$
    }
  if$
}

FUNCTION {tie.or.space.prefix}
{ duplicate$ text.length$ #3 <
    { "~" }
    { " " }
  if$
  swap$
}

FUNCTION {get.bbl.editor}
{ editor num.names$ #1 > 'bbl.editors 'bbl.editor if$ }

FUNCTION {space.word}{ " " swap$ * " " * }


% Field Conditioners, Converters, Checkers and External Interfaces

FUNCTION {empty.field.to.null.string}
{ duplicate$ empty$
    { pop$ "" }
    { skip$ }
  if$
}

FUNCTION {either.or.check}
{ empty$
    { pop$ }
    { "can't use both " swap$ * " fields in " * cite$ * warning$ }
  if$
}

FUNCTION {empty.entry.warn}
{ author empty$ title empty$
  month empty$ year empty$ note empty$ url empty$
  and and and and and and
    { "all relevant fields are empty in " cite$ * warning$ }
    'skip$
  if$
}


% The bibinfo system provides a way for the electronic parsing/acquisition
% of a bibliography's contents as is done by ReVTeX. For example, a field
% could be entered into the bibliography as:
% \bibinfo{volume}{2}
% Only the "2" would show up in the document, but the LaTeX \bibinfo command
% could do additional things with the information. IEEEtran.bst does provide
% a \bibinfo command via "\providecommand{\bibinfo}[2]{#2}". However, it is
% currently not used as the bogus bibinfo functions defined here output the
% entry values directly without the \bibinfo wrapper. The bibinfo functions
% themselves (and the calls to them) are retained for possible future use.
% 
% bibinfo.check avoids acting on missing fields while bibinfo.warn will
% issue a warning message if a missing field is detected. Prior to calling
% the bibinfo functions, the user should push the field value and then its
% name string, in that order.

FUNCTION {bibinfo.check}
{ swap$ duplicate$ missing$
    { pop$ pop$ "" }
    { duplicate$ empty$
        { swap$ pop$ }
        { swap$ pop$ }
      if$
    }
  if$
}

FUNCTION {bibinfo.warn}
{ swap$ duplicate$ missing$
    { swap$ "missing " swap$ * " in " * cite$ * warning$ pop$ "" }
    { duplicate$ empty$
        { swap$ "empty " swap$ * " in " * cite$ * warning$ }
        { swap$ pop$ }
      if$
    }
  if$
}


% The IEEE separates large numbers with more than 4 digits into groups of
% three. The IEEE uses a small space to separate these number groups. 
% Typical applications include patent and page numbers.

% number of consecutive digits required to trigger the group separation.
FUNCTION {large.number.trigger}{ #5 }

% For numbers longer than the trigger, this is the blocksize of the groups.
% The blocksize must be less than the trigger threshold, and 2 * blocksize
% must be greater than the trigger threshold (can't do more than one
% separation on the initial trigger).
FUNCTION {large.number.blocksize}{ #3 }

% What is actually inserted between the number groups.
FUNCTION {large.number.separator}{ "\," }

% So as to save on integer variables by reusing existing ones, numnames
% holds the current number of consecutive digits read and nameptr holds
% the number that will trigger an inserted space.
FUNCTION {large.number.separate}
{ 't :=
  ""
  #0 'numnames :=
  large.number.trigger 'nameptr :=
  { t empty$ not }
  { t #-1 #1 substring$ is.num
      { numnames #1 + 'numnames := }
      { #0 'numnames := 
        large.number.trigger 'nameptr :=
      }
    if$
    t #-1 #1 substring$ swap$ *
    t #-2 global.max$ substring$ 't :=
    numnames nameptr =
      { duplicate$ #1 nameptr large.number.blocksize - substring$ swap$
        nameptr large.number.blocksize - #1 + global.max$ substring$
        large.number.separator swap$ * *
        nameptr large.number.blocksize - 'numnames :=
        large.number.blocksize #1 + 'nameptr :=
      }
      { skip$ }
    if$
  }
  while$
}

% Converts all single dashes "-" to double dashes "--".
FUNCTION {n.dashify}
{ large.number.separate
  't :=
  ""
    { t empty$ not }
    { t #1 #1 substring$ "-" =
        { t #1 #2 substring$ "--" = not
            { "--" *
              t #2 global.max$ substring$ 't :=
            }
            {   { t #1 #1 substring$ "-" = }
                { "-" *
                  t #2 global.max$ substring$ 't :=
                }
              while$
            }
          if$
        }
        { t #1 #1 substring$ *
          t #2 global.max$ substring$ 't :=
        }
      if$
    }
  while$
}


% This function detects entries with names that are identical to that of
% the previous entry and replaces the repeated names with dashes (if the
% "is.dash.repeated.names" user control is nonzero).
FUNCTION {name.or.dash}
{ 's :=
   oldname empty$
     { s 'oldname := s }
     { s oldname =
         { is.dash.repeated.names
              { repeated.name.dashes }
              { s 'oldname := s }
            if$
         }
         { s 'oldname := s }
       if$
     }
   if$
}

% Converts the number string on the top of the stack to
% "numerical ordinal form" (e.g., "7" to "7th"). There is
% no artificial limit to the upper bound of the numbers as the
% two least significant digits determine the ordinal form.
FUNCTION {num.to.ordinal}
{ duplicate$ #-2 #1 substring$ "1" =
      { bbl.th * }
      { duplicate$ #-1 #1 substring$ "1" =
          { bbl.st * }
          { duplicate$ #-1 #1 substring$ "2" =
              { bbl.nd * }
              { duplicate$ #-1 #1 substring$ "3" =
                  { bbl.rd * }
                  { bbl.th * }
                if$
              }
            if$
          }
        if$
      }
    if$
}

% If the string on the top of the stack begins with a number,
% (e.g., 11th) then replace the string with the leading number
% it contains. Otherwise retain the string as-is. s holds the
% extracted number, t holds the part of the string that remains
% to be scanned.
FUNCTION {extract.num}
{ duplicate$ 't :=
  "" 's :=
  { t empty$ not }
  { t #1 #1 substring$
    t #2 global.max$ substring$ 't :=
    duplicate$ is.num
      { s swap$ * 's := }
      { pop$ "" 't := }
    if$
  }
  while$
  s empty$
    'skip$
    { pop$ s }
  if$
}

% Converts the word number string on the top of the stack to
% Arabic string form. Will be successful up to "tenth".
FUNCTION {word.to.num}
{ duplicate$ "l" change.case$ 's :=
  s "first" =
    { pop$ "1" }
    { skip$ }
  if$
  s "second" =
    { pop$ "2" }
    { skip$ }
  if$
  s "third" =
    { pop$ "3" }
    { skip$ }
  if$
  s "fourth" =
    { pop$ "4" }
    { skip$ }
  if$
  s "fifth" =
    { pop$ "5" }
    { skip$ }
  if$
  s "sixth" =
    { pop$ "6" }
    { skip$ }
  if$
  s "seventh" =
    { pop$ "7" }
    { skip$ }
  if$
  s "eighth" =
    { pop$ "8" }
    { skip$ }
  if$
  s "ninth" =
    { pop$ "9" }
    { skip$ }
  if$
  s "tenth" =
    { pop$ "10" }
    { skip$ }
  if$
}


% Converts the string on the top of the stack to numerical
% ordinal (e.g., "11th") form.
FUNCTION {convert.edition}
{ duplicate$ empty$ 'skip$
    { duplicate$ #1 #1 substring$ is.num
        { extract.num
          num.to.ordinal
        }
        { word.to.num
          duplicate$ #1 #1 substring$ is.num
            { num.to.ordinal }
            { "edition ordinal word " quote$ * edition * quote$ *
              " may be too high (or improper) for conversion" * " in " * cite$ * warning$
            }
          if$
        }
      if$
    }
  if$
}




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LATEX BIBLIOGRAPHY CODE %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FUNCTION {start.entry}
{ newline$
  "\bibitem{" write$
  cite$ write$
  "}" write$
  newline$
  ""
  initialize.prev.this.status
}

% Here we write out all the LaTeX code that we will need. The most involved
% code sequences are those that control the alternate interword spacing and
% foreign language hyphenation patterns. The heavy use of \providecommand
% gives users a way to override the defaults. Special thanks to Javier Bezos,
% Johannes Braams, Robin Fairbairns, Heiko Oberdiek, Donald Arseneau and all
% the other gurus on comp.text.tex for their help and advice on the topic of
% \selectlanguage, Babel and BibTeX.
FUNCTION {begin.bib}
{ "% Generated by IEEEtran.bst, version: " bst.file.version * " (" * bst.file.date * ")" *
  write$ newline$
  preamble$ empty$ 'skip$
    { preamble$ write$ newline$ }
  if$
  "\begin{thebibliography}{"  longest.label  * "}" *
  write$ newline$
  "\providecommand{\url}[1]{#1}"
  write$ newline$
  "\csname url@samestyle\endcsname"
  write$ newline$
  "\providecommand{\newblock}{\relax}"
  write$ newline$
  "\providecommand{\bibinfo}[2]{#2}"
  write$ newline$
  "\providecommand{\BIBentrySTDinterwordspacing}{\spaceskip=0pt\relax}"
  write$ newline$
  "\providecommand{\BIBentryALTinterwordstretchfactor}{"
  ALTinterwordstretchfactor * "}" *
  write$ newline$
  "\providecommand{\BIBentryALTinterwordspacing}{\spaceskip=\fontdimen2\font plus "
  write$ newline$
  "\BIBentryALTinterwordstretchfactor\fontdimen3\font minus \fontdimen4\font\relax}"
  write$ newline$
  "\providecommand{\BIBforeignlanguage}[2]{{%"
  write$ newline$
  "\expandafter\ifx\csname l@#1\endcsname\relax"
  write$ newline$
  "\typeout{** WARNING: IEEEtran.bst: No hyphenation pattern has been}%"
  write$ newline$
  "\typeout{** loaded for the language `#1'. Using the pattern for}%"
  write$ newline$
  "\typeout{** the default language instead.}%"
  write$ newline$
  "\else"
  write$ newline$
  "\language=\csname l@#1\endcsname"
  write$ newline$
  "\fi"
  write$ newline$
  "#2}}"
  write$ newline$
  "\providecommand{\BIBdecl}{\relax}"
  write$ newline$
  "\BIBdecl"
  write$ newline$
}

FUNCTION {end.bib}
{ newline$ "\end{thebibliography}" write$ newline$ }

FUNCTION {if.url.alt.interword.spacing}
{ is.use.alt.interword.spacing
    { is.use.url
        { url empty$ 'skip$ {"\BIBentryALTinterwordspacing" write$ newline$} if$ }
        { skip$ }
      if$
    }
    { skip$ }
  if$
}

FUNCTION {if.url.std.interword.spacing}
{ is.use.alt.interword.spacing
    { is.use.url
        { url empty$ 'skip$ {"\BIBentrySTDinterwordspacing" write$ newline$} if$ }
        { skip$ }
      if$
    }
    { skip$ }
  if$
}




%%%%%%%%%%%%%%%%%%%%%%%%
%% LONGEST LABEL PASS %%
%%%%%%%%%%%%%%%%%%%%%%%%

FUNCTION {initialize.longest.label}
{ "" 'longest.label :=
  #1 'number.label :=
  #0 'longest.label.width :=
}

FUNCTION {longest.label.pass}
{ type$ "ieeetranbstctl" =
    { skip$ }
    { number.label int.to.str$ 'label :=
      number.label #1 + 'number.label :=
      label width$ longest.label.width >
        { label 'longest.label :=
          label width$ 'longest.label.width :=
        }
        { skip$ }
      if$
    }
  if$
}




%%%%%%%%%%%%%%%%%%%%%
%% FORMAT HANDLERS %%
%%%%%%%%%%%%%%%%%%%%%

%% Lower Level Formats (used by higher level formats)

FUNCTION {format.names}
{ 'bibinfo :=
  duplicate$ empty$ 'skip$ {
  this.to.prev.status
  this.status.std
  's :=
  "" 't :=
  #1 'nameptr :=
  s num.names$ 'numnames :=
  numnames 'namesleft :=
    { namesleft #0 > }
    { s nameptr
      name.format.string
      format.name$
      bibinfo bibinfo.check
      't :=
      nameptr #1 >
        { nameptr num.names.shown.with.forced.et.al #1 + =
          numnames max.num.names.before.forced.et.al >
          is.forced.et.al and and
            { "others" 't :=
              #1 'namesleft :=
            }
            { skip$ }
          if$
          namesleft #1 >
            { ", " * t do.name.latex.cmd * }
            { s nameptr "{ll}" format.name$ duplicate$ "others" =
                { 't := }
                { pop$ }
              if$
              t "others" =
                { " " * bbl.etal emphasize * }
                { numnames #2 >
                    { "," * }
                    { skip$ }
                  if$
                  bbl.and
                  space.word * t do.name.latex.cmd *
                }
              if$
            }
          if$
        }
        { t do.name.latex.cmd }
      if$
      nameptr #1 + 'nameptr :=
      namesleft #1 - 'namesleft :=
    }
  while$
  cap.status.std
  } if$
}

%% author/editor names
FUNCTION {format.authors}{ author "author" format.names }

FUNCTION {format.editors}
{ editor "editor" format.names duplicate$ empty$ 'skip$
    { ", " *
      get.bbl.editor
      capitalize
      *
    }
  if$
}



%% date

FUNCTION {format.date}
{
  month "month" bibinfo.check duplicate$ empty$
  year  "year" bibinfo.check duplicate$ empty$
    { swap$ 'skip$
        { this.to.prev.status
          this.status.std
          cap.status.std
         "there's a month but no year in " cite$ * warning$ }
      if$
      *
    }
    { this.to.prev.status
      this.status.std
      cap.status.std
      swap$ 'skip$
        {
          swap$
          " " * swap$
        }
      if$
      *
    }
  if$
}

FUNCTION {format.booktitle}
{ booktitle duplicate$ empty$ 'skip$
    { this.to.prev.status
      this.status.std
    }
  if$
  "booktitle" bibinfo.check
  duplicate$ empty$ 'skip$
    { is.last.char.not.punct
        { punct.std 'this.status.punct := }
        { punct.no 'this.status.punct := }
      if$
      select.language
      quote.remove
      cap.status.std
    }
  if$
}

FUNCTION {format.title}
{ title duplicate$ empty$ 'skip$
    { this.to.prev.status
      this.status.std
    }
  if$
  "title" bibinfo.check
  duplicate$ empty$ 'skip$
    { is.last.char.not.punct
        { punct.std 'this.status.punct := }
        { punct.no 'this.status.punct := }
      if$
      select.language
      quote.remove
      cap.status.std
    }
  if$
}

FUNCTION {format.title.electronic}
{ title duplicate$ empty$ 'skip$
    { this.to.prev.status
      this.status.std
      cap.status.std
      "t" change.case$
    }
  if$
  "title" bibinfo.check
  duplicate$ empty$ 
    { skip$ } 
    { select.language }
  if$
}

FUNCTION {format.publisher}
{ publisher duplicate$ empty$ 'skip$
    { this.to.prev.status
      this.status.std
      "t" change.case$
    }
  if$
  "publisher" bibinfo.check
  duplicate$ empty$ 'skip$
    { is.last.char.not.punct
        { punct.std 'this.status.punct := }
        { punct.no 'this.status.punct := }
      if$
      select.language
      quote.remove
      cap.status.std
    }
  if$
}

%% journal
FUNCTION {format.journal}
{ journal duplicate$ empty$ 'skip$
    { this.to.prev.status
      this.status.std
      cap.status.std
      select.language
    }
  if$
}

%% volume/number/pages
FUNCTION {format.vol.num.pages}
{ volume boldface field.or.null
  number empty$
    'skip$
    { " (" number * ")" * *
      volume empty$
	{ "there's a number but no volume in " cite$ * warning$ }
	'skip$
      if$
    }
  if$
  pages empty$
    'skip$
    { duplicate$ empty$
	{ pop$ format.pages }
	{ " " * pages n.dashify * }
      if$
    }
  if$
}

% The intended use of format.paper is for paper numbers of inproceedings.
% The paper type can be overridden via the type field.
% We allow the type to be displayed even if the paper number is absent
% for things like "postdeadline paper"
FUNCTION {format.paper}
{ is.use.paper
     { paper empty$
        { type empty$
            { "" }
            { this.to.prev.status
              this.status.std
              type "type" bibinfo.check
              cap.status.std
            }
          if$
        }
        { this.to.prev.status
          this.status.std
          type empty$
            { bbl.paper }
            { type "type" bibinfo.check }
          if$
          " " * paper
          "paper" bibinfo.check
          *
          cap.status.std
        }
      if$
     }
     { "" } 
   if$
}


FUNCTION {format.pages}
{ pages duplicate$ empty$ 'skip$
    { this.to.prev.status
      this.status.std
      duplicate$ is.multiple.pages
        {
          bbl.pages swap$
          n.dashify
        }
        {
          bbl.page swap$
        }
      if$
      tie.or.space.prefix
      "pages" bibinfo.check
      * *
      cap.status.std
    }
  if$
}

%% note

FUNCTION {format.note}
{ note duplicate$ empty$ 'skip$
    { this.to.prev.status
      this.status.std
      punct.comma 'this.status.punct :=
      cap.status.std
      select.language
    }
  if$
}

%% URL
FUNCTION {format.url}
{ is.use.url
    { url empty$
      { "" }
      { this.to.prev.status
        this.status.std
        cap.no 'status.cap :=
        name.url.prefix " " *
        "\url{" * url * "}" *
        punct.no 'this.status.punct :=
        space.normal 'this.status.space :=
        space.normal 'prev.status.space :=
        quote.no 'this.status.quote :=
        punct.comma 'this.status.punct :=
      }
    if$
    }
    { "" }
  if$
}



%%%%%%%%%%%%%%%%%%%%
%% ENTRY HANDLERS %%
%%%%%%%%%%%%%%%%%%%%
FUNCTION {article}
{ std.status.using.comma
  start.entry
  if.url.alt.interword.spacing
  format.authors "author" output.warn
  name.or.dash
  format.title "title" output.warn
  format.journal "journal" bibinfo.check "journal" output.warn
  format.vol.num.pages output
  format.date "year" output.warn
  format.note "note" bibinfo.check "note" output.warn
  fin.entry
  if.url.std.interword.spacing
}

FUNCTION {book}
{ std.status.using.comma
  start.entry
  if.url.alt.interword.spacing
  author empty$
    { format.editors "author and editor" output.warn }
    { format.authors output.nonnull }
  if$
  name.or.dash
  format.title "title" output.warn
  author empty$
    { skip$ }
    { format.editors output }
  if$
  format.publisher "publisher" output.warn
  format.vol.num.pages output
  format.note "note" bibinfo.check "note" output.warn
  format.date "year" output.warn
  fin.entry
  if.url.std.interword.spacing
}

FUNCTION {electronic}
{ std.status.using.comma
  start.entry
  if.url.alt.interword.spacing
  format.authors output
  name.or.dash
  format.title.electronic output
  format.date output
  format.url output
  format.note "note" bibinfo.check "note" output.warn
  fin.entry
  if.url.std.interword.spacing
}

FUNCTION {proceedings}
{ std.status.using.comma
  start.entry
  if.url.alt.interword.spacing
  format.authors "author" output.warn
  name.or.dash
  format.title "title" output.warn
  format.booktitle "booktitle" output.warn
  format.editors output
  format.vol.num.pages output
  format.paper output
  format.note "note" bibinfo.check "note" output.warn
  format.url output
  format.date output
  fin.entry
  if.url.std.interword.spacing
}

FUNCTION {misc}
{ std.status.using.comma
  start.entry
  if.url.alt.interword.spacing
  format.authors output
  name.or.dash
  format.title output
  format.pages output
  format.date output
  format.note "note" bibinfo.check "note" output.warn
  format.url output
  fin.entry
  empty.entry.warn
  if.url.std.interword.spacing
}


%%%%%%%%%%%%%%%%%%
%% MAIN PROGRAM %%
%%%%%%%%%%%%%%%%%%

READ

EXECUTE {initialize.controls}
EXECUTE {initialize.status.constants}
EXECUTE {banner.message}

EXECUTE {initialize.longest.label}
ITERATE {longest.label.pass}

EXECUTE {begin.bib}
ITERATE {call.type$}
EXECUTE {end.bib}

EXECUTE{completed.message}


%% That's all folks, mds.