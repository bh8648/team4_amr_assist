import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import WorkerApp from './WorkerApp.jsx'

createRoot(document.getElementById('root')).render(
  <StrictMode><WorkerApp /></StrictMode>,
)
