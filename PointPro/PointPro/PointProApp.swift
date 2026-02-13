//
//  PointProApp.swift
//  PointPro
//
//  Created by Arshad on 13/02/26.
//

import SwiftUI

@main
struct PointProApp: App {
    @StateObject private var sessionStore = ScanSessionStore()

    var body: some Scene {
        WindowGroup {
            SessionsHomeView()
                .environmentObject(sessionStore)
        }
    }
}
