import boto
from logger import EC2Logger

"""
Requires AWS_ACCESS_KEY_ID and AWS_SECRET_ACCESS_KEY to be set as environment variables
"""

AMI_ID = 'ami-2685754f'
TIER = 'c1.xlarge'
KEY = 'sameep-surgical'
logger = EC2Logger("EC2Tools", conslogger=False)
SPOT_INSTANCE_PRICE = 1.68


class EC2Tools():
  def __init__(self, ami_id=AMI_ID, tier=TIER, key_pair=KEY):
    self.ami_id = ami_id
    self.tier = tier
    self.key_pair = key_pair
    self.num_instances = 0 
    self.conn = boto.connect_ec2()
    self.describeInstances()
  def runInstance(self, num=1, spot_instance=True, tier=None):
    if tier == None:
      tier = self.tier
    self.num_instances += 1
    image = self.conn.get_image(self.ami_id)
    logger.info("Launching new instance: [image_id = %s, tier = %s]" % 
        (self.ami_id, tier))
    if spot_instance:
      self.conn.request_spot_instances(SPOT_INSTANCE_PRICE, AMI_ID, 
          key_name=KEY, instance_type=tier)
    else:
      image.run(min_count=num, max_count=num, placement='us-east-1d', instance_type=tier, key_name=self.key_pair) 
  def terminateInstance(self, instance_id):
    logger.info("Terminating instance " + instance_id)

    self.num_instances -= len(self.conn.terminate_instances([instance_id]))
  def getAllInstances(self):
    logger.info("Generating list of all instances with image_id = " + 
        self.ami_id)
    reservations = self.conn.get_all_instances()
    instances = [ ]
    for reservation in reservations:
      for instance in reservation.instances:
        instances.append(instance)
    return instances
  def describeInstances(self):
    instances = self.getAllInstances() 
    runningInstances = [i for i in instances if i.state=="running" and i.image_id==self.ami_id]
    self.num_instances = len(runningInstances)
    return runningInstances

if __name__ == "__main__":
  ec2tools = EC2Tools()
  print ec2tools.describeInstances()
  s = raw_input("Would you like to run an instance (ex. TIER=t1.micro)? AMI = %s\n--> " % (AMI_ID))
  s = s.split('=')
  if s[0] == "TIER":
    ec2tools.runInstance(tier=s[1])
