import { useState } from 'react'

import Canvas from './components/Canvas.tsx'

function App() {
  return (
    <div className="h-screen w-full flex justify-center items-center">
      <div className="w-fit h-fit flex flex-col">
        <h1 className="w-full text-center text-2xl mb-5">Cat Leg Movement</h1>
        <Canvas/>
      </div>
    </div>
  )
}

export default App
