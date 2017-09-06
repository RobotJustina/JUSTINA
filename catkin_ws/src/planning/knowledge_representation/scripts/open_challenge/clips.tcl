
# Procedures

proc send_data { data address port} {
        set sockid [socket $address $port]
        puts $sockid "$data"
        close $sockid
}


# Input Window
toplevel .input

# Labels Inputs
label .input.label_num_rules -text  "Num. Rules"
label .input.label_path_execution -text  "Clips execution Path"
label .input.label_path_clips -text  "Clips files Path"
label .input.label_clips_files -text  "Clips files"
label .input.label_in_port -text  "Input Port"

# Entries Input
entry .input.num_rules -width 3 -textvariable num_rules -foreground green -background blue
.input.num_rules insert 0 "1"
entry .input.path_execution -width 30 -textvariable path_execution -foreground green -background blue
.input.path_execution insert 0 "/home/savage/tkclips/"
entry .input.path_clips -width 70 -textvariable path_clips -foreground green -background blue
.input.path_clips insert 0 "/home/savage/materias/robots_moviles/clips/ViRbot_cubos/"
#.input.path_clips insert 0 "/home/savage/materias/robots_moviles/clips/example_zapatos_cubos/"
entry .input.clips_files -width 10 -textvariable clips_files -foreground green -background blue
.input.clips_files insert 0 "cubes.dat"
entry .input.in_port -width 5 -textvariable in_port -foreground green -background blue
.input.in_port insert 0 "2005"

# Execute Buttons 

button .input.b0 -text LOAD -activebackground red -activeforeground orange -command {
	
	set Clips "clips_embedded_tcp_tcl"
	set file "$path_clips$clips_files"
	set command "$path_execution$Clips"

	#/home/savage/tkclips/clips_embedded_tcp_tcl -i 2005 -e cubes_original_tcp_tcl.dat -t 1 -w 1 -r 1 -n 1
	puts "exec gnome-terminal --working-directory=$path_clips -e $command -e $file -n $num_rules -t $trace -w $watch_facts -r $watch_rules -i $in_port"
	exec gnome-terminal --working-directory=$path_clips -e "$command -e $file -n $num_rules -t $trace -w $watch_facts -r $watch_rules -i $in_port"

}


button .input.b1 -text TRACE -activebackground red -activeforeground orange -command {

	set message "CONTINUE"
        #puts "$message"
	set tcp_port_out $in_port
	set tcp_address_out "localhost"
        send_data $message $tcp_address_out $tcp_port_out
}

button .input.b2 -text WATCH_RULES -activebackground red -activeforeground orange -command {

        set message "RULES"
        #puts "$message"
        set tcp_port_out $in_port
        set tcp_address_out "localhost"
        send_data $message $tcp_address_out $tcp_port_out

	set watch_rules 1
}

button .input.b3 -text WATCH_FACTS -activebackground red -activeforeground orange -command {

        set message "FACTS"
        #puts "$message"
        set tcp_port_out $in_port
        set tcp_address_out "localhost"
        send_data $message $tcp_address_out $tcp_port_out
	set watch_facts 1
}

button .input.b4 -text UNWATCH_ALL -activebackground red -activeforeground orange -command {

        set message "UNWATCH"
        #puts "$message"
        set tcp_port_out $in_port
        set tcp_address_out "localhost"
        send_data $message $tcp_address_out $tcp_port_out
	set watch_facts 0
	set watch_rules 0
}

button .input.b5 -text ENABLE_TRACE -activebackground red -activeforeground orange -command {

        set message "TRACE"
        #puts "$message"
        set tcp_port_out $in_port
        set tcp_address_out "localhost"
        send_data $message $tcp_address_out $tcp_port_out
	set trace 1
}

button .input.b6 -text UNTRACE -activebackground red -activeforeground orange -command {

        set message "UNTRACE"
        #puts "$message"
        set tcp_port_out $in_port
        set tcp_address_out "localhost"
        send_data $message $tcp_address_out $tcp_port_out
	set trace 0
}

button .input.b7 -text RESET -activebackground red -activeforeground orange -command {

        set message "RESET"
        #puts "$message"
        set tcp_port_out $in_port
        set tcp_address_out "localhost"
        send_data $message $tcp_address_out $tcp_port_out
}

button .input.b8 -text CHANGE_NUM_RULES -activebackground red -activeforeground orange -command {

        set message "NUMBER $num_rules"
        #puts "$message"
        set tcp_port_out $in_port
        set tcp_address_out "localhost"
        send_data $message $tcp_address_out $tcp_port_out
}


# Grid Input
grid config .input.label_num_rules -column 0 -row 0 -sticky e
grid config .input.num_rules -column 1 -row 0 -sticky snew
grid config .input.b8 -column 2 -row 0 -sticky snew
grid config .input.label_path_execution -column 3 -row 0 -sticky e
grid config .input.path_execution -column 4 -row 0 -sticky snew
grid config .input.label_path_clips -column 0 -row 1 -sticky e
grid config .input.path_clips -column 1 -row 1 -sticky snew
grid config .input.label_clips_files -column 2 -row 1 -sticky e
grid config .input.clips_files -column 3 -row 1 -sticky snew
grid config .input.b0 -column 4 -row 1 -sticky snew
grid config .input.label_in_port -column 0 -row 2 -sticky e
grid config .input.in_port -column 1 -row 2 -sticky snew
grid config .input.b4 -column 2 -row 2 -sticky snew
grid config .input.b5 -column 3 -row 2 -sticky snew
grid config .input.b2 -column 0 -row 3 -sticky snew
grid config .input.b3 -column 1 -row 3 -sticky snew
grid config .input.b6 -column 2 -row 3 -sticky snew
grid config .input.b7 -column 3 -row 3 -sticky snew
grid config .input.b1 -column 4 -row 3 -sticky snew




# it sets the check buttons

set trace 1
set watch_facts 1
set watch_rules 1

checkbutton .check1 -text "Load Trace" -variable trace
checkbutton .check2 -text "Load Watch Facts" -variable watch_facts
checkbutton .check3 -text "Load Watch Rules" -variable watch_rules

pack .check1 .check2 .check3 

