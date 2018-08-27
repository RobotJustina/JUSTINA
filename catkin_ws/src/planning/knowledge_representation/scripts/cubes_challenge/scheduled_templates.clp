;********************************************************
;*							*
;*	scheduled_templates.clp				*
;*							*
;*							*
;*			University of Mexico		*
;*			Jesus Savage-Carmona		*
;*                      Adrian Revueltas                *
;*			Mauricio Matamoros		*
;*							*
;*			1/23/14				*
;*							*
;********************************************************


(deftemplate state
        (field name
                 (type SYMBOL)
                 (default nil)
        )
	(field number
                (type NUMBER)
                (default 1)
        )
	(field duration
                (type NUMBER)
                (default 1)
        )
	(field status
                 (type SYMBOL)
                 (default inactive)
        )

)




(deftemplate plan
        (field name
                 (type SYMBOL)
                 (default nil)
        )
        (field number
                (type NUMBER)
                (default 1)
        )
	(multifield actions
		(type SYMBOL)
	)
        (field duration
                (type NUMBER)
                (default 1)
        )
        (field status
                 (type SYMBOL)
                 (default inactive)
        )
	(
	 field statusTwo
		(type SYMBOL)
		(default active)
	)
	(multifield actions_num_params
		(type NUMBER)
		(default 0)
	)

)




(deftemplate condition
	(field conditional 
                (type SYMBOL)
		(default if)
        )
	(multifield arguments 
                (type SYMBOL)
        )
	(field true-state 
                (type NUMBER)
                (default 1)
        )
	(field false-state 
                (type NUMBER)
                (default 1)
        )
        (field name-scheduled
                (type SYMBOL)
                (default nil)
        )
        (field state-number
                 (type NUMBER)
                 (default 0)
        )
)


(deftemplate cd-task
	(field cd
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
	(field message
                 (type STRING)
                 (default "")
        )
	(field part
                 (type SYMBOL)
                 (default nil)
        )
        (field dir
                 (type SYMBOL)
                 (default nil)
        )
        (field distance
                 (type NUMBER)
                 (default 0)
        )
        (field angle
                 (type NUMBER)
                 (default 0)
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
        (field answer
                 (type SYMBOL)
                 (default nil)
        )
        (field state-number
                 (type NUMBER)
                 (default 0)
        )
        (field name-scheduled
                (type SYMBOL)
                (default nil)
        )
)


