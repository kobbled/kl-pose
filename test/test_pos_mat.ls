/PROG TEST_POS_MAT
/ATTR
COMMENT = "variable list";
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP = 1,1,1,*,*;
/MN
 : ! ***************** ;
 : ! Test pose function ;
 : ! converting matrix ;
 : ! representation ;
 : ! to cartesian ;
 : ! ***************** ;
 :  ;
 : ! Set Frames ;
 : UTOOL_NUM=1 ;
 : UFRAME_NUM=3 ;
 :  ;
 : ! get tool frame in matrix ;
 : ! representation ;
 : PR[85:pos]=UTOOL[1] ;
 : ! convert to cartesian format ;
 : CALL POSE_CNVCART(85,1) ;
/END
