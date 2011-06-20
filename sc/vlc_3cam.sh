vlc http://localhost:8080/stream?topic=/prosilica1/image_raw --sout="#duplicate{dst=std{access=file,mux=mp4,dst='/home/ziang/Desktop/stream1.mp4'},dst=nodisplay}" &
vlc http://localhost:8080/stream?topic=/prosilica2/image_raw --sout="#duplicate{dst=std{access=file,mux=mp4,dst='/home/ziang/Desktop/stream2.mp4'},dst=nodisplay}" &
vlc http://localhost:8080/stream?topic=/prosilica3/image_raw --sout="#duplicate{dst=std{access=file,mux=mp4,dst='/home/ziang/Desktop/stream3.mp4'},dst=nodisplay}" &

