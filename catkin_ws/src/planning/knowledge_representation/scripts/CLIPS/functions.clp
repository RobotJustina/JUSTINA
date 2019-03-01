(deffunction waitsec
        (?time)
	(bind ?sym (gensym))
        (python-call setTimer ?time ?sym)
)

