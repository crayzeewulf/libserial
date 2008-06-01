<!DOCTYPE style-sheet PUBLIC "-//James Clark//DTD DSSSL Style Sheet//EN" [
<!ENTITY % html "IGNORE">
<![%html;[
<!ENTITY % print "IGNORE">
<!ENTITY docbook.dsl PUBLIC "-//Norman Walsh//DOCUMENT DocBook HTML Stylesheet//EN" CDATA dsssl>
]]>
<!ENTITY % print "INCLUDE">
<![%print;[
<!ENTITY docbook.dsl PUBLIC "-//Norman Walsh//DOCUMENT DocBook Print Stylesheet//EN" CDATA dsssl>
]]>
]>
<style-sheet>
<style-specification id="print" use="docbook">
<style-specification-body> 


;;
;; Paper/Page Characteristics
;;
(define %left-margin%   1.0in)
(define %right-margin%  1.0in)
(define %top-margin%    1.0in)
(define %bottom-margin% 1.0in)
;;
;; What size paper do you need? A4, USletter, USlandscape, or RedHat?
;;
(define %paper-type%    "USletter")
;;
;; Quadding
;;
(define %default-quadding% 'justify)

;;
;; Bibliography
;;
(define biblio-number
  ;; Enumerate bibliography entries
  #t)

;;
;; Footnotes
;;
(define bop-footnotes 
;; Put footnotes at the bottom of the page
  #t)

;;
;; Generate table of contents.
;;
(define *toc-depth* 2)
(define (toc-depth nd) *toc-depth*)
(define %generate-article-toc% #t)

;;
;; Put the table of contents on the title page.
;;
(define %generate-article-toc-on-titlepage% #f)
(define %generate-article-titlepage-on-separate-page% #f)

;;
;; Fonts
;;
(define %smaller-size-factor% 
  ;; Smaller font scaling factor
  0.8)
;;
;;What font would you like for the body?
(define %body-font-family% 
 "Palatino")

;;What size do you want the body fonts?
(define %bf-size%
 (case %visual-acuity%
    (("tiny") 8pt)
    (("normal") 10pt)
    (("presbyopic") 12pt)
    (("large-type") 24pt)))

;;
;; Numbering
;;
(define %section-autolabel% 
  ;; Are sections enumerated?
  #t)

;; 
;; Rules
;;
(define %figure-rules%
  ;; Specify rules before and after an Figure
  #f)

;;
;; Put figure title below the figures.
;;
(define ($object-titles-after$)
  ;; List of objects who's titles go after the object
  (list (normalize "figure")))

(define %hyphenation%
  ;; Allow automatic hyphenation?
  #t)

</style-specification-body>
</style-specification>
<style-specification id="html" use="docbook">
<style-specification-body> 

;; customize the html stylesheet

</style-specification-body>
</style-specification>
<external-specification id="docbook" document="docbook.dsl">
</style-sheet>
