(define (problem p-planetaryExploration) 

    (:domain d-planetaryExploration)
  
    (:objects 
        P0404 P3508 P1530 P3636 P0905 - coord
        slow fast - rover_speed
     	rover1 - rover
    )

    (:init
        
        (= (dur_pict) 10) (= (pict_battery_use) 20)
        (= (dur_dril) 18) (= (dril_battery_use) 20)
        (= (dur_comm) 11) (= (comm_battery_use) 20)
        (= (dur_anal) 21) (= (anal_battery_use) 20)
        (= (dur_rech) 15) 
     
        (IN P0404 rover1)
     
        (= (max_battery rover1) 100)  (= (current_battery rover1) 100)  (= (low_battery rover1) 20)
        
        (= (speed slow rover1) 5)  (= (speed fast rover1) 10)
     
     	(= (distance P0404 P3508) 10)  (= (distance P1530 P0404) 20)  (= (distance P0905 P0404) 10)
     	(= (distance P0404 P1530) 20)  (= (distance P1530 P3508) 10)  (= (distance P0905 P3508) 40)
     	(= (distance P0404 P3636) 10)  (= (distance P1530 P3636) 30)  (= (distance P0905 P1530) 10)
     	(= (distance P0404 P0905) 30)  (= (distance P1530 P0905) 10)  (= (distance P0905 P3636) 25)
     
     	(= (distance P3508 P0404) 10)  (= (distance P3636 P0404) 10)
     	(= (distance P3508 P1530) 10)  (= (distance P3636 P3508) 10)
     	(= (distance P3508 P3636) 10)  (= (distance P3636 P1530) 10)
     	(= (distance P3508 P0905) 40)  (= (distance P3636 P0905) 25)
     
     	(DANGER P0905)
     	
    )

    (:goal (and 
                (COMMUNICATION P1530 rover1)
                (ANALYSIS P1530 rover1)
                (COMMUNICATION P3508 rover1)
                (DRILL P3508 rover1)
                (ANALYSIS P0404 rover1)
                (DRILL P0404 rover1)
                (COMMUNICATION P3636 rover1)
                (PICTURE P3636 rover1)
            )        
    )
  
    (:constraints (and (preference keepSafe (always (forall (?rover - rover ?cord - coord) (imply (DANGER ?cord) (not (IN ?cord ?rover))))))
                       ;(preference phot56   (sometime (PICTURE P0404 rover1)))
                    ))
)
