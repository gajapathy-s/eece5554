;; Auto-generated. Do not edit!


(when (boundp 'gnss::Customgps)
  (if (not (find-package "GNSS"))
    (make-package "GNSS"))
  (shadow 'Customgps (find-package "GNSS")))
(unless (find-package "GNSS::CUSTOMGPS")
  (make-package "GNSS::CUSTOMGPS"))

(in-package "ROS")
;;//! \htmlinclude Customgps.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gnss::Customgps
  :super ros::object
  :slots (_header _latitude _longitude _altitude _utm_easting _utm_northing _LatitudeDir _LongitudeDir _UTC _zone _letter _hdop _gpgga_read ))

(defmethod gnss::Customgps
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:utm_easting __utm_easting) 0.0)
    ((:utm_northing __utm_northing) 0.0)
    ((:LatitudeDir __LatitudeDir) "")
    ((:LongitudeDir __LongitudeDir) "")
    ((:UTC __UTC) 0.0)
    ((:zone __zone) 0)
    ((:letter __letter) "")
    ((:hdop __hdop) 0.0)
    ((:gpgga_read __gpgga_read) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _utm_easting (float __utm_easting))
   (setq _utm_northing (float __utm_northing))
   (setq _LatitudeDir (string __LatitudeDir))
   (setq _LongitudeDir (string __LongitudeDir))
   (setq _UTC (float __UTC))
   (setq _zone (round __zone))
   (setq _letter (string __letter))
   (setq _hdop (float __hdop))
   (setq _gpgga_read (string __gpgga_read))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:utm_easting
   (&optional __utm_easting)
   (if __utm_easting (setq _utm_easting __utm_easting)) _utm_easting)
  (:utm_northing
   (&optional __utm_northing)
   (if __utm_northing (setq _utm_northing __utm_northing)) _utm_northing)
  (:LatitudeDir
   (&optional __LatitudeDir)
   (if __LatitudeDir (setq _LatitudeDir __LatitudeDir)) _LatitudeDir)
  (:LongitudeDir
   (&optional __LongitudeDir)
   (if __LongitudeDir (setq _LongitudeDir __LongitudeDir)) _LongitudeDir)
  (:UTC
   (&optional __UTC)
   (if __UTC (setq _UTC __UTC)) _UTC)
  (:zone
   (&optional __zone)
   (if __zone (setq _zone __zone)) _zone)
  (:letter
   (&optional __letter)
   (if __letter (setq _letter __letter)) _letter)
  (:hdop
   (&optional __hdop)
   (if __hdop (setq _hdop __hdop)) _hdop)
  (:gpgga_read
   (&optional __gpgga_read)
   (if __gpgga_read (setq _gpgga_read __gpgga_read)) _gpgga_read)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; float64 _altitude
    8
    ;; float64 _utm_easting
    8
    ;; float64 _utm_northing
    8
    ;; string _LatitudeDir
    4 (length _LatitudeDir)
    ;; string _LongitudeDir
    4 (length _LongitudeDir)
    ;; float64 _UTC
    8
    ;; uint8 _zone
    1
    ;; string _letter
    4 (length _letter)
    ;; float64 _hdop
    8
    ;; string _gpgga_read
    4 (length _gpgga_read)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _utm_easting
       (sys::poke _utm_easting (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _utm_northing
       (sys::poke _utm_northing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _LatitudeDir
       (write-long (length _LatitudeDir) s) (princ _LatitudeDir s)
     ;; string _LongitudeDir
       (write-long (length _LongitudeDir) s) (princ _LongitudeDir s)
     ;; float64 _UTC
       (sys::poke _UTC (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _zone
       (write-byte _zone s)
     ;; string _letter
       (write-long (length _letter) s) (princ _letter s)
     ;; float64 _hdop
       (sys::poke _hdop (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _gpgga_read
       (write-long (length _gpgga_read) s) (princ _gpgga_read s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _altitude
     (setq _altitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _utm_easting
     (setq _utm_easting (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _utm_northing
     (setq _utm_northing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _LatitudeDir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _LatitudeDir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _LongitudeDir
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _LongitudeDir (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _UTC
     (setq _UTC (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _zone
     (setq _zone (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _letter
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _letter (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _hdop
     (setq _hdop (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _gpgga_read
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _gpgga_read (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get gnss::Customgps :md5sum-) "40332f7c8344b73c529c712b74b53deb")
(setf (get gnss::Customgps :datatype-) "gnss/Customgps")
(setf (get gnss::Customgps :definition-)
      "Header header 
float64 latitude 
float64 longitude 
float64 altitude 
float64 utm_easting 
float64 utm_northing 
string   LatitudeDir
string   LongitudeDir
float64 UTC
uint8 zone 
string letter 
float64 hdop 
string gpgga_read

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :gnss/Customgps "40332f7c8344b73c529c712b74b53deb")


