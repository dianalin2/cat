import { useContext, useEffect, useState } from 'react';
import { RosContext } from './RosProvider';
import { Topic } from 'roslib';

import Canvas from './components/Canvas.tsx'

interface EncoderFeedback {
  [key: string]: {
    position: number;
  }
}

interface EncoderFeedbackMessage {
  name: string[];
  position: number[];
}

function App() {
  const { ros, isConnected } = useContext(RosContext);
  const [feedback, setFeedback] = useState<EncoderFeedback | null>(null);

  useEffect(() => {
    if (ros && isConnected) {
      console.log('ROS is connected');
    } else {
      console.log('ROS is not connected');
      return;
    }

    const feedbackListener = new Topic({
      ros: ros,
      name: '/encoder_feedback',
      messageType: 'msgs/msg/EncoderFeedback',
    })

    feedbackListener.subscribe((message: unknown) => {
      const typedMessage = message as EncoderFeedbackMessage;
      console.log('Received feedback:', message);

      const feedbackData: EncoderFeedback = {};
      for (let i = 0; i < typedMessage.name.length; i++) {
        const name = typedMessage.name[i];
        const value = typedMessage.position[i];
        feedbackData[name] = { position: value };
      }

      setFeedback(feedbackData);
    });
  }, [isConnected]);

  return (
    <div className="h-screen w-full flex justify-center items-center">
      <div className="w-fit h-fit flex flex-col">
        <h1 className="w-full text-center text-2xl mb-5">Cat Leg Movement</h1>
        {feedback && (
          <div className="w-full text-center mb-5">
            <pre className="bg-gray-100 p-2 rounded">{
              Object.entries(feedback).map(([name, data]) => `${name}: ${data.position}`).join(', ')
            }</pre>
          </div>
        )}
        <Canvas />
      </div>
    </div >
  )
}

export default App
