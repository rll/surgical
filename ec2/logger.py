import logging, time

class EC2Logger():
  def __init__(self, name, conslogger=True):
    logger_format = '%(asctime)s %(name)s %(levelname)-8s %(message)s'
    logger_date_format = '%a, %d %b %Y %H:%M:%S'
    if name == "MASTER" or name == "EC2Tools":
      filepath = "logs/ec2-"
    elif name == "sc":
      filepath = "logs/sc-"
    else:
      filepath = "/home/ubuntu/code/trunk/surgical/ec2/logs/ec2-"
    logging.basicConfig(level=logging.DEBUG,
      format=logger_format,
      datefmt=logger_date_format,
      filename=filepath + name + "-" + str(int(time.time())) + '.log',
      filemode='w')
    if conslogger:
      console = logging.StreamHandler()
      console.setLevel(logging.INFO)    
      formatter=logging.Formatter(logger_format, datefmt=logger_date_format)
      console.setFormatter(formatter)
      logging.getLogger('').addHandler(console)
    self.logger = logging.getLogger(name)
    logging.getLogger('boto').setLevel(logging.CRITICAL)
  def info(self, msg):
    self.logger.info(msg)
  def error(self, msg):
    self.logger.error(msg)
  def debug(self, msg):
    self.logger.debug(msg)
  def critical(self, msg):
    self.logger.critical(msg)
  def warn(self, msg):
    self.logger.warn(msg)

if __name__ == "__main__":
  logger = EC2Logger('testscript')
  logger.info("Hello world")
  logger.error("Goodbye world")
