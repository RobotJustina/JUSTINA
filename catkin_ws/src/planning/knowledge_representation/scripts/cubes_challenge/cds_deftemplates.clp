
;************************************************
;*						*
;*	cds_deftemplates definitions		*
;*						*
;*			J.Savage, UNAM		*
;*                      1/24/14                 *
;*                      1/24/14                 *
;*						*
;************************************************

(deftemplate atrans
	(field actor
		 (type SYMBOL)
		 (default nil)
	)
	(field obj
		 (type SYMBOL)
		 (default nil)
	)
	(field from
		 (type SYMBOL)
		 (default nil)
	)
 	(field to
		 (type SYMBOL)
		 (default nil)
	)
)

(deftemplate ptrans
	(field actor
		 (type SYMBOL)
		 (default nil)
	)
	(field obj
		 (type SYMBOL)
		 (default nil)
	)
	(field from
		 (type SYMBOL)
		 (default nil)
	)
 	(field to
		 (type SYMBOL)
		 (default nil)
	)
	(field num
		(type NUMBER)
		(default 0)
	)
)


(deftemplate mtrans
	(field actor
		 (type SYMBOL)
		 (default nil)
	)
	(field obj
		 (type SYMBOL)
		 (default nil)
	)
	(field from
		 (type SYMBOL)
		 (default nil)
	)
 	(field to
		 (type SYMBOL)
		 (default nil)
	)
)

(deftemplate attend
	(field actor
		 (type SYMBOL)
		 (default nil)
	)
	(field obj
		 (type SYMBOL)
		 (default nil)
	)
	(field from
		 (type SYMBOL)
		 (default nil)
	)
 	(field to
		 (type SYMBOL)
		 (default nil)
	)
)




(deftemplate grab
	(field actor
		 (type SYMBOL)
		 (default nil)
	)
	(field obj
		 (type SYMBOL)
		 (default nil)
	)
	(field from
		 (type SYMBOL)
		 (default nil)
	)
 	(field to
		 (type SYMBOL)
		 (default nil)
	)
)


(deftemplate release
        (field actor
                 (type SYMBOL)
                 (default nil)
        )
        (field obj
                 (type SYMBOL)
                 (default nil)
        )
        (field from
                 (type SYMBOL)
                 (default nil)
        )
        (field to
                 (type SYMBOL)
                 (default nil)
        )
)
                                                                                                                         


(deftemplate state
	(field verb
                 (type SYMBOL)
                 (default nil)
        )
        (field name
                 (type SYMBOL)
                 (default nil)
        )
        (field object
                 (type SYMBOL)
                 (default nil)
        )
	(field human
                 (type SYMBOL)
                 (default nil)
        )
	(field place
		(type SYMBOL)
                 (default nil)
        )
	(field color
                (type SYMBOL)
                 (default nil)
        )
	(field image
                (type SYMBOL)
                 (default nil)
        )
        (field value
                 (type SYMBOL)
                 (default nil)
        )
        (field number
                 (type NUMBER)
                 (default 1)
        )
)


(deftemplate qtrans
	(field actor
		 (type SYMBOL)
		 (default nil)
	)
	(field question
		 (type SYMBOL)
		 (default nil)
	)
 	(field aux-verb
		 (type SYMBOL)
		 (default nil)
	)
	(field verb
		 (type SYMBOL)
		 (default nil)
	)
	(field human
		 (type SYMBOL)
		 (default nil)
	)
	(field obj
		 (type SYMBOL)
		 (default nil)
	)
	(field answer
		 (type SYMBOL)
		 (default nil)
	)
)


(deftemplate memory
        (field action 
                 (type SYMBOL)
                 (default nil)
        )
        (multifield parameters
                 (type SYMBOL)
                 (default nil)
        )
        (field master
                 (type SYMBOL)
                 (default nil)
        )
	(field num
                 (type NUMBER)
                 (default 1)
        )
)

(deftemplate planner
	(field name
                 (type SYMBOL)
                 (default nil)
        )
	(field action
                 (type SYMBOL)
                 (default nil)
        )
	(field actor
		 (type SYMBOL)
		 (default nil)
	)
	(field obj
		 (type SYMBOL)
		 (default nil)
	)
	(field from
		 (type SYMBOL)
		 (default nil)
	)
 	(field to
		 (type SYMBOL)
		 (default nil)
	)
	(multifield num
		(type NUMBER)
		(default 0 0)
	)
)

(deftemplate into
	 (field name 
                 (type SYMBOL)
                 (default nil)
        ) 
        (field number
                (type NUMBER)
                (default 1)
        )
	(field next
		(type NUMBER)
		(default 2)
	)
        (field plan
                (type NUMBER)
		(default 0)
        )
        (field status
                 (type SYMBOL)
                 (default inactive)
        )
)

(deftemplate pile
	(field name
		(type SYMBOL)
		(default nil)
	)
	(multifield first_stack
		(type SYMBOL)
		(default nil)
	)
	(multifield second_stack
		(type SYMBOL)
		(default nil)
	)
	(field status
		(type SYMBOL)
		(default nil)
	)
	(field number
		(type NUMBER)
		(default 0)
	)
)


