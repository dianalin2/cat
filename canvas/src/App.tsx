import { useContext, useEffect, useState } from 'react';
import { RosContext } from './RosProvider';
import { Topic } from 'roslib';

import Canvas from './components/Canvas.tsx'

function App() {
  const { ros, isConnected } = useContext(RosContext);
  const [feedback, setFeedback] = useState<any>(null);

  useEffect(() => {
    if (ros && isConnected) {
      console.log('ROS is connected');
    } else {
      console.log('ROS is not connected');
      return;
    }

    const feedbackListener = new Topic({
      ros: ros,
      name: '/serial_data',
      messageType: 'msgs/msg/SerialMessage',
    })

    feedbackListener.subscribe((message) => {
      console.log('Received feedback:', message);
      setFeedback(message);
    });
  }, [isConnected]);

  return (
    <div className="h-screen w-full flex justify-center items-center">
      <div className="w-fit h-fit flex flex-col">
        <h1 className="w-full text-center text-2xl mb-5">Cat Leg Movement</h1>
        {feedback && (
          <div className="w-full text-center mb-5">
            <p>Received Feedback:</p>
            <pre className="bg-gray-100 p-2 rounded">{Array.from(atob(feedback.payload)).map((byte, index) => (
              <span key={index}>{byte.charCodeAt(0).toString(16).padStart(2, '0')}</span>
            ))}</pre>
          </div>
        )}
        <Canvas />
      </div>
    </div>
  )
}

export default App
