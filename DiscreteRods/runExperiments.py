from subprocess import call

# Main Method    
if __name__ == '__main__':
  status = call(["./runExperiment", "out3", "c12", "c12_world", "500", "1000", "5", "0"])
  print status
