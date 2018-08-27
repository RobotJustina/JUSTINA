
;************************************************
;*						*
;*	objects_deftemplates definitions	*
;*						*
;*			J.Savage, UNAM		*
;*			1/24/14			*
;*			1/24/14			*
;*						*
;************************************************


(deftemplate item
        (field type
                 (type SYMBOL)
                 (default nil)
        )
        (field name
                 (type SYMBOL)
                 (default nil)
        )
        (multifield status
                 (type SYMBOL)
                 (default nil)
        )
        (multifield attributes
                 (type SYMBOL)
                 (default nil)
        )
        (multifield pose
                 (type NUMBER)
                 (default 0 0 0)
        )
        (field grasp
                 (type NUMBER)
                 (default 1)
        )
        (field zone
                 (type SYMBOL)
                 (default nil)
        )
        (field possession
                 (type SYMBOL)
                 (default nobody)
        )
        (field image
                 (type SYMBOL)
                 (default nil)
        )
        (field script
                 (type SYMBOL)
                 (default nil)
        )
        (field num
                 (type NUMBER)
                 (default 1)
        )
        (field shared
                 (type SYMBOL)
                 (default false)
        )
	(multifield zones
                 (type SYMBOL)
                 (default nil)
        )
	(multifield hands
                 (type SYMBOL)
                 (default nil)
        )
        (multifield category
                 (type SYMBOL)
                 (default nil)
        )
        (field room
                 (type SYMBOL)
                 (default nil)
        )
        (field weight
                 (type NUMBER)
                 (default 0)
        )
        (field size
                 (type NUMBER)
                 (default 0)
        )
        (field wide
                (type NUMBER)
                (default 0)
        )
        (field height
                (type NUMBER)
                (default 0)
        )
        (field color
                 (type SYMBOL)
                 (default nil)
        )
        (field quantity
                 (type NUMBER)
                 (default 0)
        )
        (field quantitys
                 (type NUMBER)
                 (default 0)
        )
        (field biggest
                (type SYMBOL)
                (default nil)
        )
        (field smallest
                (type SYMBOL)
                (default nil)
        )
        (field heaviest
                (type SYMBOL)
                (default nil)
        )
        (field lightest
                (type SYMBOL)
                (default nil)
        )

)




(deftemplate Human
	(field name
		 (type SYMBOL)
		 (default nil)
	)
	(multifield objs
		 (type SYMBOL)
		 (default nil)
	)
	(field room
		(type SYMBOL)
                 (default livingroom)
        )
	(field zone
		 (type SYMBOL)
		 (default nil)
	)
	(field status
                 (type SYMBOL)
                 (default nil)
        )
	(multifield pose
                 (type NUMBER)
                 (default 0 0 0)
        )
	(field shared
                 (type SYMBOL)
                 (default false)
        )

	
)


(deftemplate crowd
	(field name
		 (type SYMBOL)
		 (default nil)
	)
	(field pose
		 (type SYMBOL)
		 (default nil)
	)
	(field gender
		(type SYMBOL)
                 (default nil)
        )
	(field gesture
		 (type SYMBOL)
		 (default nil)
	)
)


(deftemplate Robot
	(field name
		 (type SYMBOL)
		 (default nil)
	)
	(field room
                (type SYMBOL)
                 (default livingroom)
        )
	(multifield hands
		 (type SYMBOL)
		 (default nil)
	)
	(multifield tray
		 (type SYMBOL)
		 (default nil)
	)
	(field manipulator
		 (type SYMBOL)
		 (default lower)
	)
	(field zone
		 (type SYMBOL)
		 (default nil)
	)
	(field status
                 (type SYMBOL)
                 (default nil)
        )
	(multifield pose
                 (type NUMBER)
                 (default 0 0 0)
        )
	(field shared
                 (type SYMBOL)
                 (default false)
        )

 )


(deftemplate Objects
	(field name
		 (type SYMBOL)
		 (default nil)
	)
	(field status
                 (type SYMBOL)
                 (default nil)
        )
	(multifield attributes
		 (type SYMBOL)
		 (default nil)
	)
	(multifield pose
                 (type NUMBER)
                 (default 0 0 0)
        )
	(field grasp
		 (type SYMBOL)
		 (default nil)
	)
	(field zone
		 (type SYMBOL)
		 (default nil)
	)
	(field possession
		 (type SYMBOL)
		 (default nobody)
	)
	(field image
                 (type SYMBOL)
                 (default nil)
        )
	(field script
		 (type SYMBOL)
		 (default nil)
	)
	(field num
		 (type NUMBER)
		 (default 1)
	)
	(field shared
                 (type SYMBOL)
                 (default false)
        )

 )


(deftemplate Door
	(field name
		 (type SYMBOL)
		 (default nil)
	)
	(field status
                 (type SYMBOL)
                 (default nil)
        )
	(multifield attributes
		 (type SYMBOL)
		 (default nil)
	)
	(field grasp
		 (type SYMBOL)
		 (default nil)
	)
	(field zone
		 (type SYMBOL)
		 (default nil)
	)
	(field possession
		 (type SYMBOL)
		 (default nobody)
	)
	(field image
                 (type SYMBOL)
                 (default nil)
        )
	(field script
		 (type SYMBOL)
		 (default nil)
	)
	(field num
		 (type NUMBER)
		 (default 1)
	)
	(field shared
                 (type SYMBOL)
                 (default false)
        )

 )


(deftemplate Room
	(field name
		 (type SYMBOL)
		 (default nil)
	)
	(field status
                 (type SYMBOL)
                 (default nil)
        )
	(multifield attributes
		 (type SYMBOL)
		 (default nil)
	)
	(field grasp
		 (type SYMBOL)
		 (default nil)
	)
	(multifield zones
		 (type SYMBOL)
		 (default nil)
	)
	(field possession
		 (type SYMBOL)
		 (default nobody)
	)
	(field image
                 (type SYMBOL)
                 (default nil)
        )
	(field script
		 (type SYMBOL)
		 (default nil)
	)
	(field num
		 (type NUMBER)
		 (default 1)
	)
	(field shared
                 (type SYMBOL)
                 (default false)
        )

 )



(deftemplate Furniture
	(field name
		 (type SYMBOL)
		 (default nil)
	)
	(field status
		 (type SYMBOL)
		 (default nil)
	)
	(multifield attributes
		 (type SYMBOL)
		 (default nil)
	)
	(field grasp
		 (type SYMBOL)
		 (default nil)
	)
	(field zone
		 (type SYMBOL)
		 (default nil)
	)
	(field possession
		 (type SYMBOL)
		 (default nobody)
	)
	(field image
                 (type SYMBOL)
                 (default nil)
        )
	(field script
		 (type SYMBOL)
		 (default nil)
	)
	(field num
		 (type NUMBER)
		 (default 1)
	)
	(field shared
                 (type SYMBOL)
                 (default false)
        )

 )


(deftemplate Arm
        (field name
                 (type SYMBOL)
                 (default nil)
        )
        (field status
                 (type SYMBOL)
                 (default nil)
        )
        (multifield attributes
                 (type SYMBOL)
                 (default nil)
        )
        (field possession
                 (type SYMBOL)
                 (default nobody)
        )
        (field grasp
                 (type SYMBOL)
                 (default nil)
        )
        (field bandera
                 (type SYMBOL)
                 (default nil)
        )
)


