#include "ObjectTrajectoryRecorder.h"

ObjectTrajectoryRecorder::ObjectTrajectoryRecorder()
{
	sprintf(_fileName, "%s.txt", SAVED_BASE_NAME);
}

ObjectTrajectoryRecorder::ObjectTrajectoryRecorder(const char* fileName)
{
	sprintf(_fileName, "%s.txt", fileName);
}

void ObjectTrajectoryRecorder::setFileName(const char* fileName)
{
	sprintf(_fileName, "%s.txt", fileName);
}

void ObjectTrajectoryRecorder::writeObjectsToFile(World* world)
{
  std::cout << "Writing to: " << _fileName << std::endl;

  ofstream file;
  file.precision(20);
  file.open(_fileName);

  vector<EnvObject*> objects = *(world->getEnvObjs());
  vector<ThreadConstrained*> threads = *(world->getThreads());

  for (int i = 0; i < threads.size(); i++) {
    threads[i]->writeToFile(file);
  }
  
  for (int i = 0; i < objects.size(); i++) {
  	objects[i]->updateIndFromPointers(world);
    objects[i]->writeToFile(file);
  }
  
  file << NO_OBJECT << " ";
  file << "\n";

  file.close();

  std::cout << "Writing Done\n";
}
